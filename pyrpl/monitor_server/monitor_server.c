 /* COPYRIGHT NOTICE OF MONITOR.C 
 * $Id$
 *
 * @brief Simple program to read/write from/to any location in memory.
 *
 * @Author Crt Valentincic <crt.valentincic@redpitaya.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */
/*
###############################################################################
#    pyrplockbox - DSP servo controller for quantum optics with the RedPitaya
#    Copyright (C) 2014-2016  Leonhard Neuhaus  (neuhaus@spectro.jussieu.fr)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
############################################################################### 
 */



 
/* 
Communication protocol for the data server:

The program is launched on the redpitaya with 

./monitor-server PORT-NUMBER, where the default port number is 2222.  

We allow for bidirectional data transfer. The client (python program) connects to the server, which in return accepts the connection. 
The client sends 8 bytes of data:
Byte 1 is interpreted as a character: 'r' for read and 'w' for write, and 'c' for close. All other messages are ignored. 
Byte 2 is reserved. 
Bytes 3+4 are interpreted as unsigned int. This number n is the amount of 4-byte-units to be read or written. Maximum is 2^16. 
Bytes 5-8 are the start address to be written to. 

If the command is read, the server will then send the requested 4*n bytes to the client. 
If the command is write, the server will wait for 4*n bytes of data from the server and write them to the designated FPGA address space. 
If the command is close, or if the connection is broken, the server program will terminate. 

After this, the server will wait for the next command. 
*/
 
 /* for now the program is utterly unoptimized... */
 
#define _GNU_SOURCE


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <time.h>

void error(const char *msg);

#define FATAL do { fprintf(stderr,"Error at line %d, file %s (%d) [%s]\n", __LINE__, __FILE__, errno, strerror(errno)); \
									error("FATAL ERROR"); exit(1); } while(0)
 
//#define MAP_SIZE 4096UL
//#define MAP_SIZE 65536UL
#define MAP_SIZE 131072UL
//allowed address space: 0x40000000 to 0x40800000 has size 0x800000 = 128*65536 = 8388608
//#define MAP_SIZE 8388608UL
#define MAP_MASK (MAP_SIZE - 1)
#define MAX_LENGTH 65535

#define DEBUG_MONITOR 0

unsigned long read_value(unsigned long a_addr);
unsigned long* read_values(unsigned long a_addr, unsigned long* a_values_buffer, unsigned long a_len);
void write_value(unsigned long a_addr, unsigned long a_value);
void write_values(unsigned long a_addr, unsigned long* a_values, unsigned long a_len);

//FPGA memory handlers
void* map_base = (void*)(-1);
int fd = -1;

//sockets are globally defined for error handling
int sockfd;
int newsockfd;

/* -------------------------------------------------------------------------
 * DMA point-cloud multicast thread
 *
 * Polls the FPGA write pointer at 0x40A00000, reads complete packets from
 * the ring buffer at 0x1e000000, and multicasts each packet via UDP using
 * sendmsg(iovec) to avoid a copy across the ring-buffer wrap boundary.
 *
 * Packet layout (HIST_BLOCK_SIZE+1 words of 8 bytes each):
 *   Word 0  [63:60] channel_id  [35+HSZ:32+HSZ] fmt_version
 *           [31+HSZ:32] fft_hist_index  [31:0] frame_cnt
 *   Word 1..N: { peak_bin_down, peak_bin_up }  (each IDX = FSZ+FRAC bits)
 *
 * This thread forwards the payload verbatim and only depends on packet *size*;
 * the field widths live in the FPGA descriptor regs (0x170/0x194) that the host
 * reads. The packet size itself is discovered at runtime from the DMA register
 * slot (dma_reg[1] = HIST_BLOCK_SIZE), so it tracks the FPGA build with no
 * recompile; DMA_PKT_WORDS_DEFAULT is only a fallback for an old/garbage read.
 * ------------------------------------------------------------------------- */
#define DMA_BUF_BASE         0x1e000000UL
#define DMA_BUF_WORDS        16384       /* must match BUF_WORDS in dma_s2mm */
#define DMA_BUF_BYTES        (DMA_BUF_WORDS * 8)
#define DMA_REG_BASE         0x40a00000UL /* AXI-Lite slot 5: [0]=wr_ptr [1]=hist_block */
#define DMA_PKT_WORDS_DEFAULT 184        /* fallback: 1 header + 183 data = 1472 B = one MTU */
#define DMA_DEFAULT_MCAST "239.255.0.1"
#define DMA_DEFAULT_PORT  12468
#define DMA_DEFAULT_UNI_PORT 12466       /* unicast workaround for multicast-unfriendly hosts */

typedef struct { int port; const char *group; } dma_thread_args_t;

/* Unicast point-cloud target. A multi-homed host (e.g. Windows with several
 * virtual NICs) often joins the multicast group on the wrong interface and
 * never receives anything. As a workaround the DMA thread ALSO unicasts every
 * packet to the host that opened the TCP register link — discovered from the
 * accept() source address, so no host-side IP config is needed. g_uni_ip is 0
 * (disabled) until a register client connects. Both are written once from the
 * main thread and only read by the DMA thread, so plain volatiles suffice. */
static volatile uint32_t g_uni_ip   = 0;                    /* dest, network order; 0 = off */
static volatile uint16_t g_uni_port = DMA_DEFAULT_UNI_PORT;  /* dest, host order */
/* Local UDP port the DMA socket binds: where the point cloud egresses from AND
 * where host registration datagrams arrive. Keeping send and recv on one port
 * is what makes the NAT hole-punch work — the board replies to a NAT'd host
 * along the exact mapping that host's registration opened. */
static volatile uint16_t g_uni_listen_port = DMA_DEFAULT_UNI_PORT;

static void *dma_poll_thread(void *arg)
{
    dma_thread_args_t *a = (dma_thread_args_t *)arg;
    int port = a->port;
    const char *mcast_group = a->group;
    int devfd;
    volatile uint64_t *dma_buf;
    volatile uint32_t *dma_reg;
    int sock;
    struct sockaddr_in mcast_addr;
    uint32_t rd_ptr = 0;
    const struct timespec poll_sleep = { .tv_sec = 0, .tv_nsec = 100000 }; /* 100 µs */

    devfd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (devfd < 0) { perror("dma: open /dev/mem"); return NULL; }

    dma_buf = mmap(NULL, DMA_BUF_BYTES, PROT_READ, MAP_SHARED, devfd, DMA_BUF_BASE);
    if (dma_buf == MAP_FAILED) {
        perror("dma: mmap ring buffer"); close(devfd); return NULL;
    }

    dma_reg = mmap(NULL, 4096, PROT_READ, MAP_SHARED, devfd, DMA_REG_BASE);
    if (dma_reg == MAP_FAILED) {
        perror("dma: mmap register"); munmap((void *)dma_buf, DMA_BUF_BYTES);
        close(devfd); return NULL;
    }

    /* Discover packet size from the FPGA: dma_reg[1] = HIST_BLOCK_SIZE (data
     * words per packet); full packet = +1 header word. Fall back to the default
     * on an out-of-range read (e.g. an older bitstream without this register). */
    uint32_t hist_block = dma_reg[1] & 0xffff;
    uint32_t pkt_words;
    if (hist_block == 0 || hist_block >= DMA_BUF_WORDS) {
        fprintf(stderr, "dma: HIST_BLOCK_SIZE read %u out of range, using default %d\n",
                hist_block, DMA_PKT_WORDS_DEFAULT - 1);
        pkt_words = DMA_PKT_WORDS_DEFAULT;
    } else {
        pkt_words = hist_block + 1;
    }
    uint32_t pkt_bytes = pkt_words * 8;
    fprintf(stderr, "dma: packet = %u words (%u bytes)\n", pkt_words, pkt_bytes);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("dma: socket"); goto cleanup; }

    {
        unsigned char ttl = 1;
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    }

    /* Bind the unicast/registration port so the point cloud egresses from it and
     * host hole-punch registrations land here. */
    {
        struct sockaddr_in bind_addr;
        int reuse = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family      = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port        = htons(g_uni_listen_port);
        if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
            perror("dma: bind unicast port");
    }

    memset(&mcast_addr, 0, sizeof(mcast_addr));
    mcast_addr.sin_family      = AF_INET;
    mcast_addr.sin_port        = htons((uint16_t)port);
    mcast_addr.sin_addr.s_addr = inet_addr(mcast_group);

    fprintf(stderr, "dma: multicasting on %s:%d (unicast workaround on port %d)\n",
            mcast_group, port, (int)g_uni_port);

    while (1) {
        /* Pick up host registration datagrams (NAT hole-punch). Any datagram on
         * this port re-points the unicast stream at its post-NAT source — for a
         * NAT'd host that's the gateway's IP and the mapped port, reachable only
         * this way. Drains non-blocking so the send loop never stalls. */
        {
            struct sockaddr_in src;
            socklen_t slen = sizeof(src);
            char rbuf[64];
            while (recvfrom(sock, rbuf, sizeof(rbuf), MSG_DONTWAIT,
                            (struct sockaddr *)&src, &slen) > 0) {
                g_uni_ip   = src.sin_addr.s_addr;
                g_uni_port = ntohs(src.sin_port);
            }
        }

        uint32_t wr_ptr = dma_reg[0] & (DMA_BUF_WORDS - 1);
        uint32_t avail  = (wr_ptr - rd_ptr + DMA_BUF_WORDS) & (DMA_BUF_WORDS - 1);

        if (avail < pkt_words) {
            nanosleep(&poll_sleep, NULL);
            continue;
        }

        /* scatter-gather across possible ring-buffer wrap — no memcpy */
        uint32_t end = (rd_ptr + pkt_words) & (DMA_BUF_WORDS - 1);
        struct iovec iov[2];
        int niov;

        if (rd_ptr + pkt_words <= DMA_BUF_WORDS) {
            iov[0].iov_base = (void *)&dma_buf[rd_ptr];
            iov[0].iov_len  = pkt_bytes;
            niov = 1;
        } else {
            uint32_t first  = DMA_BUF_WORDS - rd_ptr;
            iov[0].iov_base = (void *)&dma_buf[rd_ptr];
            iov[0].iov_len  = first * 8;
            iov[1].iov_base = (void *)&dma_buf[0];
            iov[1].iov_len  = (pkt_words - first) * 8;
            niov = 2;
        }

        struct msghdr msg = {
            .msg_name    = &mcast_addr,
            .msg_namelen = sizeof(mcast_addr),
            .msg_iov     = iov,
            .msg_iovlen  = niov,
        };
        sendmsg(sock, &msg, 0);

        /* Also unicast to the register client's host, if one has connected.
         * Same iov (the ring-buffer payload is still valid — rd_ptr has not
         * advanced yet). Lets multicast-unfriendly hosts get the point cloud. */
        uint32_t uni_ip = g_uni_ip;
        if (uni_ip) {
            struct sockaddr_in uni_addr;
            memset(&uni_addr, 0, sizeof(uni_addr));
            uni_addr.sin_family      = AF_INET;
            uni_addr.sin_port        = htons(g_uni_port);
            uni_addr.sin_addr.s_addr = uni_ip;
            struct msghdr umsg = {
                .msg_name    = &uni_addr,
                .msg_namelen = sizeof(uni_addr),
                .msg_iov     = iov,
                .msg_iovlen  = niov,
            };
            sendmsg(sock, &umsg, 0);
        }

        rd_ptr = end;
    }

cleanup:
    munmap((void *)dma_buf, DMA_BUF_BYTES);
    munmap((void *)dma_reg, 4096);
    close(devfd);
    return NULL;
}

//open and close memory mapping to FPGA registers
void open_map_base() {
    int addr = 0x40000000;
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
	if(map_base == (void *) -1) FATAL;
}

void close_map_base() {
	/*if (map_base != (void*)(-1)) {
		if(munmap(map_base, MAP_SIZE) == -1) FATAL;
		map_base = (void*)(-1);
	}
	if (fd != -1) {
		close(fd);
	}
	*/;
}
/*
//basic read and write operations
unsigned long* read_values(unsigned long a_addr, unsigned long* a_values_buffer, unsigned long a_len) {
	unsigned long* virt_addr = map_base + (a_addr & MAP_MASK);
	unsigned long i;
	printf("address to read %d",(int)a_addr);
	for (i = 0; i < a_len; i++) {;
		a_values_buffer[i] = virt_addr[i];
	}
	return a_values_buffer;
}

void write_values(unsigned long a_addr, unsigned long* a_values, unsigned long a_len) {
	void* virt_addr = map_base + (a_addr & MAP_MASK);
	unsigned long i;
	for (i = 0; i < a_len; i++) {
		((unsigned long *) virt_addr)[i] = a_values[i];
	}
}
*/

/* server process and error handling */

void error(const char *msg)
{
    perror(msg);
    close(newsockfd); 
    close(sockfd);
	//clean up the memory mapping
	close_map_base();
    exit(-1);
}

int main(int argc, char *argv[])
{
     int portno;
	 unsigned int data_length;
	 unsigned long address;
     socklen_t clilen;

     char data_buffer[8+sizeof(unsigned long)*MAX_LENGTH];
	 unsigned long * rw_buffer =(unsigned long*)&(data_buffer[8]);
	 char* buffer = (char*)&(data_buffer[0]);
     
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }

     /* start DMA multicast thread (argv[2] = "mcast-ip:udp-port") */
     {
         static dma_thread_args_t dma_args;
         static char dma_group[64];
         int dma_port = DMA_DEFAULT_PORT;
         strncpy(dma_group, DMA_DEFAULT_MCAST, sizeof(dma_group));
         if (argc >= 3)
             sscanf(argv[2], "%63[^:]:%d", dma_group, &dma_port);
         /* Optional argv[3] overrides the unicast point-cloud port (default
          * 12466) — both the local listen/egress port and the fallback dest
          * port. A registering host later overrides the dest with its real
          * post-NAT address. */
         if (argc >= 4) {
             uint16_t up = (uint16_t)atoi(argv[3]);
             g_uni_listen_port = up;
             g_uni_port        = up;
         }
         dma_args.port  = dma_port;
         dma_args.group = dma_group;
         pthread_t dma_tid;
         pthread_create(&dma_tid, NULL, dma_poll_thread, &dma_args);
         pthread_detach(dma_tid);
     }

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
	int enable = 1;
	if (setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&enable,sizeof(int))<0)
		error("setsockopt(SO_REUSEADDR) failed");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0)
          error("ERROR on accept");
	 else {
		 /* Fallback unicast target: whoever just connected for register access
		  * (the host running the GUI), at the unicast listen port. Good enough
		  * for a directly-reachable host; a NAT'd host (e.g. WSL) instead sends a
		  * registration datagram that the DMA thread uses to learn its real
		  * post-NAT address and port, overriding this. */
		 g_uni_ip   = cli_addr.sin_addr.s_addr;
		 g_uni_port = g_uni_listen_port;
		 fprintf(stderr, "dma: unicast fallback target %s:%d (until host registers)\n",
		         inet_ntoa(cli_addr.sin_addr), (int)g_uni_port);
		 printf("Incoming client connection accepted!");
	 }
	
	//open_map_base();
	 //service loop
     while (0==0) {
		 //read next header from client
		 bzero(buffer,8);
		 n = recv(newsockfd,buffer,8,MSG_WAITALL);
		 if (n < 0) error("ERROR reading from socket");
		 if (n != 8) error("ERROR reading from socket - incorrect header length");
		 //confirm control sequence
	 ////n=send(newsockfd,buffer,8,0); 
	 ////if (n != 8) error("ERROR control sequence mirror incorreclty transmitted");
	     //interpret the header
    	 address = ((unsigned long*)buffer)[1]; //address to be read/written
		 data_length = buffer[2]+(buffer[3]<<8); //number of "unsigned long" to be read/written
		 if (data_length > MAX_LENGTH)
			 data_length = MAX_LENGTH;
		 if (data_length == 0)
			continue;
		 //test for various cases Read, Write, Close
		 else if (buffer[0] == 'r') { //read from FPGA
			read_values(address, rw_buffer, data_length);
			//send the data
			n = send(newsockfd,(void*)data_buffer,data_length*sizeof(unsigned long)+8,0);
			if (n < 0) error("ERROR writing to socket");
			if (n != data_length*sizeof(unsigned long)+8) error("ERROR wrote incorrect number of bytes to socket");
		 }
		 else if  (buffer[0] == 'w') { //write to FPGA
			//read new data from socket
			n = recv(newsockfd,(void*)rw_buffer,data_length*sizeof(unsigned long),MSG_WAITALL);
			if (n < 0) error("ERROR reading from socket");
			if (n != data_length*sizeof(unsigned long)) error("ERROR read incorrect number of bytes to socket");
			//write FPGA memory
			write_values(address, rw_buffer, data_length);
			n=send(newsockfd,buffer,8,0);
			if (n != 8) error("ERROR control sequence mirror incorreclty transmitted");
		 }
		 else if (buffer[0] == 'c') break; //close program
		 else error("ERROR unknown control character - server and client out of sync"); //if an unknown control sequence is received, terminate for security reasons
	 }
	 //close the socket
     close(newsockfd); 
	 close(sockfd);
	 //clean up the memory mapping
	 close_map_base();
	 return 0; 
}


// old version with steady reinstantiation of mmap (slow)
unsigned long* read_values(unsigned long a_addr, unsigned long* a_values_buffer, unsigned long a_len) {
    int fd = -1;
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, a_addr & ~MAP_MASK);
	if(map_base == (void *) -1) FATAL;
	
	void* virt_addr = map_base + (a_addr & MAP_MASK);
	unsigned long i;
	for (i = 0; i < a_len; i++) {
		a_values_buffer[i] = ((unsigned long*) virt_addr)[i];
	}

	if (map_base != (void*)(-1)) {
		if(munmap(map_base, MAP_SIZE) == -1) FATAL;
		map_base = (void*)(-1);
	}
	if (fd != -1) {
		close(fd);
	}
	return a_values_buffer;
}

void write_values(unsigned long a_addr, unsigned long* a_values, unsigned long a_len) {
    int fd = -1;
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, a_addr & ~MAP_MASK);
	if(map_base == (void *) -1) FATAL;
	
	void* virt_addr = map_base + (a_addr & MAP_MASK);
	unsigned long i;
	for (i = 0; i < a_len; i++) {
				((unsigned long *) virt_addr)[i] = a_values[i];
	}
	
	if (map_base != (void*)(-1)) {
		if(munmap(map_base, MAP_SIZE) == -1) FATAL;
		map_base = (void*)(-1);
	}
	if (fd != -1) {
		close(fd);
	}
}
