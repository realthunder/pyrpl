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
If the command is 'b' (bulk read of the memory window, see MEM_WIN_BASE), bytes
3+4 are the length in KB (up to 64 MB) and bytes 5-8 the start address, 16-byte
aligned and inside the window. Byte 2 caps this stream's rate in MB/s (0: no cap
of its own). The server sends the 8-byte header back, then the bytes in one
stream; a request it cannot serve gets the header back with bytes 3+4 cleared and
no data. A server without 'b' drops the connection instead.
All bulk streams together are paced to BULK_MBPS (TCP pacing, the share of each
re-evaluated per chunk), so the link keeps room for the point stream's UDP: an
unpaced stream fills a gigabit link, and point packets are lost on the way to
the host (in a Windows host's virtual switch in front of WSL).
If the command is close, or if the connection is broken, the server drops the
current client and loops back to accept() to wait for the next one (it does NOT
terminate — TCP keepalive frees a half-open connection from an unplugged host
so a GUI can reconnect without the server being restarted). The daemon is only
stopped externally (killall), e.g. when installing a new build.

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
#include <netinet/tcp.h>
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

/* Register words, protocol words and addresses are 32 bits on every target.
 * `unsigned long` is 64-bit on aarch64 (ZCU216): it would turn each register
 * access into an 8-byte AXI transfer, which the PL bridge refuses, and shift
 * the header and data layout the pyrpl client sends. */
typedef uint32_t word_t;

/* The client always addresses the Red Pitaya map at 0x4000_0000. REG_BASE is
 * where that map really is: 0x4000_0000 on the Red Pitaya, 0x8000_0000 on the
 * ZCU216 (M_AXI_HPM0_LPD; `make monitor_server_zcu216`). */
#ifndef REG_BASE
#define REG_BASE 0x40000000UL
#endif
#define RP_REG_BASE 0x40000000UL

/* Memory window: client addresses MEM_WIN_BASE .. + MEM_WIN_SIZE read and write
 * the reserved DDR from DMA_BUF_BASE on (point-cloud ring, then the ZCU216's
 * frame snapshots from 1 MB on), so the client fetches a captured frame with
 * ordinary register reads. */
#define MEM_WIN_BASE 0x48000000UL
#define MEM_WIN_SIZE 0x02000000UL
#define REG_PHYS(a) ((off_t)(((unsigned long)(a) - MEM_WIN_BASE < MEM_WIN_SIZE) \
    ? DMA_BUF_BASE + ((unsigned long)(a) - MEM_WIN_BASE)                    \
    : REG_BASE + ((unsigned long)(a) - RP_REG_BASE)))

#define DEBUG_MONITOR 0

word_t* read_values(word_t a_addr, word_t* a_values_buffer, unsigned long a_len);
void write_values(word_t a_addr, word_t* a_values, unsigned long a_len);
static int bulk_read(int sock, word_t a_addr, unsigned long a_bytes, unsigned cap_mbps,
                     char *buf, size_t buf_len);

//FPGA memory handlers
void* map_base = (void*)(-1);
int fd = -1;

//sockets are globally defined for error handling
int sockfd;
int newsockfd = -1;

/* clients are served in parallel (client_thread) */
#define MAX_CLIENTS 8
static pthread_mutex_t g_reg_lock    = PTHREAD_MUTEX_INITIALIZER;  /* register accesses */
static pthread_mutex_t g_client_lock = PTHREAD_MUTEX_INITIALIZER;
static int             g_clients     = 0;
static void *client_thread(void *arg);

/* total rate of all bulk reads ('b'), MB/s (10^6 B/s); 0: unpaced */
#ifndef BULK_MBPS
#define BULK_MBPS 80
#endif
static pthread_mutex_t g_bulk_lock   = PTHREAD_MUTEX_INITIALIZER;
static int             g_bulk_active = 0;

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
#ifndef DMA_BUF_BASE
#define DMA_BUF_BASE         0x1e000000UL
#endif
#define DMA_BUF_WORDS        16384       /* must match BUF_WORDS in dma_s2mm */
#define DMA_BUF_BYTES        (DMA_BUF_WORDS * 8)
/* [0] = wr_ptr, [1] = hist_block. Red Pitaya: slot 5. ZCU216: the point stream
 * in slot 1 (zcu216_points, -DDMA_REG_BASE=0x80200180UL); need not be page
 * aligned. */
#ifndef DMA_REG_BASE
#define DMA_REG_BASE         (REG_BASE + 0xa00000UL)
#endif
#define DMA_PKT_WORDS_DEFAULT 184        /* fallback: 1 header + 183 data = 1472 B = one MTU */
#define DMA_PAD_WORD         ((uint64_t)~0ULL)  /* all-ones idle/pad sentinel (bit63=1 but NOT a real header) */
/* Fresh-edge guard (words). wr_ptr is a PL register read directly over the GP
 * port, but the ring DATA goes PL -> HP2 -> AFI FIFO -> DDR controller: the
 * pointer can lead the data's DRAM visibility by a few words, so reading a
 * packet the moment avail >= pkt_words can pick up STALE tail words (previous
 * ring lap — observed as an old header ~3 words from the edge, which made the
 * host's seq tracker count phantom ring-capacity gaps). Only ship a packet once
 * the writer is safely past it — or once the writer has gone idle for a poll
 * cycle (100 us), after which any in-flight write has long since landed. */
#define DMA_READ_GUARD_WORDS 64
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
static volatile int      g_uni_registered = 0;  /* a host registered its address */
/* Local UDP port the DMA socket binds: where the point cloud egresses from AND
 * where host registration datagrams arrive. Keeping send and recv on one port
 * is what makes the NAT hole-punch work — the board replies to a NAT'd host
 * along the exact mapping that host's registration opened. */
static volatile uint16_t g_uni_listen_port = DMA_DEFAULT_UNI_PORT;

/* Find the ring offset of the next packet start and return it as the new
 * rd_ptr. Starts from the write pointer (fresh data) and scans candidate
 * phases p in [0, pkt_words): the true packet phase is the only one where all
 * ALIGN_K stride-pkt_words words have bit63 set. Ambiguity (an idle stream
 * full of all-ones pad words) or a glitch consumes one packet and rescans;
 * bounded, then falls back to the best-scoring phase with a warning. */
static uint32_t dma_align(volatile uint64_t *dma_buf, volatile uint32_t *dma_reg,
                          uint32_t pkt_words, const struct timespec *poll_sleep)
{
    const int ALIGN_K = 16;              /* packets sampled per phase scan */
    int scans;
    uint32_t rd = dma_reg[0] & (DMA_BUF_WORDS - 1);   /* start from fresh data */
    uint32_t best_p = 0;
    int best_score = -1;
    for (scans = 0; scans < 500; scans++) {
        uint32_t wr    = dma_reg[0] & (DMA_BUF_WORDS - 1);
        uint32_t avail = (wr - rd + DMA_BUF_WORDS) & (DMA_BUF_WORDS - 1);
        if (avail < (uint32_t)(ALIGN_K + 1) * pkt_words + DMA_READ_GUARD_WORDS) {
            nanosleep(poll_sleep, NULL);
            scans--;                     /* waiting for data is not a scan */
            continue;
        }
        uint32_t p;
        int perfect = 0;
        uint32_t perfect_p = 0;
        best_score = -1;
        for (p = 0; p < pkt_words; p++) {
            int k, score = 0;
            for (k = 0; k < ALIGN_K; k++) {
                uint32_t idx = (rd + p + (uint32_t)k * pkt_words)
                               & (DMA_BUF_WORDS - 1);
                /* A REAL header has bit63 set AND is not the all-ones pad
                 * sentinel. Counting pads as headers (they also have bit63)
                 * made a pad-run region score 16/16 at MANY phases -> the align
                 * reported "ambiguous", fell back to a wrong best guess, and
                 * thrashed slip<->realign forever (never recovering from an
                 * assembler-reset storm). Excluding pads leaves only the true
                 * framing phase scoring high. */
                uint64_t w = dma_buf[idx];
                score += ((w >> 63) & 1) && w != DMA_PAD_WORD;
            }
            if (score > best_score) { best_score = score; best_p = p; }
            if (score == ALIGN_K)   { perfect++; perfect_p = p; }
        }
        if (perfect == 1) {
            fprintf(stderr, "dma: aligned to packet framing (phase %u)\n",
                    perfect_p);
            return (rd + perfect_p) & (DMA_BUF_WORDS - 1);
        }
        /* 0 perfect: glitch/stale data; >1 perfect: pad-filled idle stream is
         * phase-ambiguous. Consume one packet and rescan — real data
         * disambiguates immediately. */
        rd = (rd + pkt_words) & (DMA_BUF_WORDS - 1);
    }
    fprintf(stderr, "dma: packet-framing phase ambiguous after %d scans; "
            "using best guess %u (score %d/%d)\n", scans, best_p, best_score,
            16);
    return (rd + best_p) & (DMA_BUF_WORDS - 1);
}

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

    {
        void *m = mmap(NULL, 4096, PROT_READ, MAP_SHARED, devfd, DMA_REG_BASE & ~0xfffUL);
        if (m == MAP_FAILED) {
            perror("dma: mmap register"); munmap((void *)dma_buf, DMA_BUF_BYTES);
            close(devfd); return NULL;
        }
        dma_reg = (volatile uint32_t *)((char *)m + (DMA_REG_BASE & 0xfffUL));
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

    /* --- Packet-framing alignment -------------------------------------------
     * The FPGA writes fixed pkt_words-long packets back to back into the ring,
     * and the FIRST word of every packet has bit63 set (a real header or the
     * all-ones pad sentinel; data words have bit63=0). Reader and writer
     * advance in lock step in stream space, so the reader's phase error is
     * constant — but ONLY between assembler resets: any FPGA-side reconfigure
     * (dma_nch change, pipeline reset, scan re-sync) can cut the in-flight
     * packet short, slipping the stream phase. A misphased reader emits
     * datagrams that begin mid-packet; the host parser must discard all data
     * words before the first header (~half of every packet at a typical
     * phase), and since the phase error is stable between resets the SAME scan
     * cells are lost every sweep — frozen point-cloud cells.
     *
     * dma_align() detects the true phase by content: the only offset p where
     * ALL stride-pkt_words words have bit63 set is the packet start. It is run
     * once at startup and again WHENEVER a chunk fails the header-first check
     * in the send loop (self-healing across FPGA reconfigures). */
    rd_ptr = dma_align(dma_buf, dma_reg, pkt_words, &poll_sleep);

    uint32_t wr_prev = (uint32_t)-1;   /* fresh-edge guard: writer-idle detector */
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
                g_uni_registered = 1;
            }
        }

        uint32_t wr_ptr = dma_reg[0] & (DMA_BUF_WORDS - 1);
        uint32_t avail  = (wr_ptr - rd_ptr + DMA_BUF_WORDS) & (DMA_BUF_WORDS - 1);

        if (avail < pkt_words) {
            wr_prev = wr_ptr;
            nanosleep(&poll_sleep, NULL);
            continue;
        }

        /* Fresh-edge guard (see DMA_READ_GUARD_WORDS): the packet at rd_ptr is
         * only safe to read once the writer is DMA_READ_GUARD_WORDS past its
         * end, or once the writer has been idle for a full poll cycle. */
        if (avail < pkt_words + DMA_READ_GUARD_WORDS && wr_ptr != wr_prev) {
            wr_prev = wr_ptr;
            nanosleep(&poll_sleep, NULL);
            continue;
        }
        wr_prev = wr_ptr;

        /* Self-healing framing check: every true packet starts with a bit63
         * word. A miss means the FPGA assembler was reset mid-packet (dma_nch
         * change, pipeline reset, scan re-sync) and the stream phase slipped —
         * re-align instead of shipping mid-packet datagrams (whose leading
         * words the host must discard, freezing the same cells every sweep). */
        if (!((dma_buf[rd_ptr] >> 63) & 1)) {
            fprintf(stderr, "dma: framing slip at rd=%u — re-aligning\n", rd_ptr);
            rd_ptr = dma_align(dma_buf, dma_reg, pkt_words, &poll_sleep);
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
    off_t addr = REG_PHYS(RP_REG_BASE);
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

/* one client: its own buffer, register accesses under g_reg_lock */
static void *client_thread(void *arg)
{
    int fd = (int)(intptr_t)arg;
    char *data_buffer = malloc(8 + sizeof(word_t) * MAX_LENGTH);
    word_t *rw_buffer = (word_t *)(data_buffer + 8);
    char *buffer = data_buffer;
    unsigned int data_length;
    word_t address;
    int n;

    while (data_buffer) {
        n = recv(fd, buffer, 8, MSG_WAITALL);
        if (n != 8) break;                       /* client gone or short read */
        memcpy(&address, buffer + 4, sizeof(address));
        data_length = (unsigned char)buffer[2] + ((unsigned char)buffer[3] << 8);
        if (data_length > MAX_LENGTH)
            data_length = MAX_LENGTH;
        if (data_length == 0)
            continue;
        else if (buffer[0] == 'r') {
            pthread_mutex_lock(&g_reg_lock);
            read_values(address, rw_buffer, data_length);
            pthread_mutex_unlock(&g_reg_lock);
            n = send(fd, data_buffer, data_length * sizeof(word_t) + 8, MSG_NOSIGNAL);
            if (n != (int)(data_length * sizeof(word_t) + 8)) break;
        }
        else if (buffer[0] == 'w') {
            n = recv(fd, rw_buffer, data_length * sizeof(word_t), MSG_WAITALL);
            if (n != (int)(data_length * sizeof(word_t))) break;
            pthread_mutex_lock(&g_reg_lock);
            write_values(address, rw_buffer, data_length);
            pthread_mutex_unlock(&g_reg_lock);
            n = send(fd, buffer, 8, MSG_NOSIGNAL);
            if (n != 8) break;
        }
        else if (buffer[0] == 'b') {             /* bulk read of the memory window */
            if (bulk_read(fd, address, (unsigned long)data_length << 10,
                          (unsigned char)buffer[1], (char *)rw_buffer, sizeof(word_t) * MAX_LENGTH) < 0)
                break;
        }
        else break;                              /* 'c', or out of sync */
    }
    close(fd);
    free(data_buffer);
    pthread_mutex_lock(&g_client_lock);
    g_clients--;
    pthread_mutex_unlock(&g_client_lock);
    return NULL;
}

int main(int argc, char *argv[])
{
     int portno;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
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
     listen(sockfd, 16);
     /* Accept loop: every client gets its own thread (up to MAX_CLIENTS at
      * once), so e.g. the pyrpl GUI, a frame-snapshot tool and a point-stream
      * tool share the board. Register accesses are serialised (g_reg_lock);
      * bulk reads of the memory window run in parallel. A client that drops
      * or closes ('c') only ends its own thread. */
     signal(SIGPIPE, SIG_IGN);   /* a vanished client must not kill the server */
     while (0==0) {
         clilen = sizeof(cli_addr);
         int cfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
         if (cfd < 0) {
              perror("ERROR on accept");
              continue;   /* keep the daemon alive; try to accept again */
         }
         /* TCP keepalive: without it, an unplugged host leaves its thread
          * blocked forever in recv() on the half-open connection. Detect a dead
          * peer in ~6s (3s idle + 3 x 1s probes). */
         {
              int ka = 1, idle = 3, intvl = 1, cnt = 3;
              setsockopt(cfd, SOL_SOCKET,  SO_KEEPALIVE,  &ka,    sizeof(ka));
              setsockopt(cfd, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
              setsockopt(cfd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
              setsockopt(cfd, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
         }
         /* Fallback unicast target: whoever connected for register access (the
          * host running the GUI), at the unicast listen port, good enough for a
          * directly-reachable host. A NAT'd host (e.g. WSL) instead sends a
          * registration datagram that the DMA thread uses to learn its real
          * post-NAT address and port; once one has come, a further connection
          * (another tool) does not move the stream back to the fallback. */
         if (!g_uni_registered) {
              g_uni_ip   = cli_addr.sin_addr.s_addr;
              g_uni_port = g_uni_listen_port;
              fprintf(stderr, "dma: unicast fallback target %s:%d (until host registers)\n",
                      inet_ntoa(cli_addr.sin_addr), (int)g_uni_port);
         }
         pthread_mutex_lock(&g_client_lock);
         int full = g_clients >= MAX_CLIENTS;
         if (!full) g_clients++;
         pthread_mutex_unlock(&g_client_lock);
         if (full) {
              fprintf(stderr, "client refused: %d clients already\n", MAX_CLIENTS);
              close(cfd);
              continue;
         }
         pthread_t tid;
         if (pthread_create(&tid, NULL, client_thread, (void *)(intptr_t)cfd) != 0) {
              perror("client thread");
              close(cfd);
              pthread_mutex_lock(&g_client_lock);
              g_clients--;
              pthread_mutex_unlock(&g_client_lock);
              continue;
         }
         pthread_detach(tid);
         fprintf(stderr, "Incoming client connection accepted (%s)\n",
                 inet_ntoa(cli_addr.sin_addr));
     }
     //not reached in normal operation (the accept loop above never exits)
     close(sockfd);
     //clean up the memory mapping
     close_map_base();
     return 0;
}


// old version with steady reinstantiation of mmap (slow)
/* Map the pages that [phys, phys + bytes) touches; *base gets the mapping, the
 * return value the address of phys inside it. A request is up to MAX_LENGTH
 * words (256 KB), larger than any fixed map size. */
static volatile word_t *map_range(int fd, off_t phys, size_t bytes, void **base, size_t *len)
{
    off_t pg = (off_t)sysconf(_SC_PAGESIZE);
    off_t start = phys & ~(pg - 1);
    *len = (size_t)((phys - start) + (off_t)bytes + pg - 1) & ~(size_t)(pg - 1);
    *base = mmap(0, *len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, start);
    if (*base == MAP_FAILED) FATAL;
    return (volatile word_t *)((char *)*base + (phys - start));
}

word_t* read_values(word_t a_addr, word_t* a_values_buffer, unsigned long a_len) {
    int fd = -1;
    void *base;
    size_t len;
    off_t phys = REG_PHYS(a_addr);
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    volatile word_t* virt_addr = map_range(fd, phys, a_len * sizeof(word_t), &base, &len);
    unsigned long i;
    for (i = 0; i < a_len; i++) {
        a_values_buffer[i] = virt_addr[i];
    }
    if (munmap(base, len) == -1) FATAL;
    close(fd);
    return a_values_buffer;
}

void write_values(word_t a_addr, word_t* a_values, unsigned long a_len) {
    int fd = -1;
    void *base;
    size_t len;
    off_t phys = REG_PHYS(a_addr);
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    volatile word_t* virt_addr = map_range(fd, phys, a_len * sizeof(word_t), &base, &len);
    unsigned long i;
    for (i = 0; i < a_len; i++) {
        virt_addr[i] = a_values[i];
    }
    if (munmap(base, len) == -1) FATAL;
    close(fd);
}

/* 'b': stream [a_addr, a_addr + a_bytes) of the memory window to the client.
 * The window (reserved, no-map DDR) is device memory to /dev/mem, so the CPU
 * has one load in flight at a time: 16-byte loads read ~134 MB/s on the ZCU216
 * (4-byte loads 35 MB/s), more than the gigabit link takes. The window stays
 * mapped; the kernel sends one chunk while the next is read. */
static int send_all(int sock, const char *p, size_t n)
{
    while (n) {
        ssize_t k = send(sock, p, n, MSG_NOSIGNAL);
        if (k <= 0) return -1;
        p += k;
        n -= (size_t)k;
    }
    return 0;
}

/* the pacing rate of one bulk stream: its share of BULK_MBPS, at most its own
 * cap (B/s; ~0U: unpaced) */
static unsigned bulk_rate(unsigned cap_mbps)
{
    unsigned long r = ~0UL;
    int n;
    pthread_mutex_lock(&g_bulk_lock);
    n = g_bulk_active;
    pthread_mutex_unlock(&g_bulk_lock);
    if (BULK_MBPS > 0 && n > 0)
        r = (unsigned long)BULK_MBPS * 1000000UL / (unsigned long)n;
    if (cap_mbps && (unsigned long)cap_mbps * 1000000UL < r)
        r = (unsigned long)cap_mbps * 1000000UL;
    return r >= ~0U ? ~0U : (unsigned)r;
}

static void bulk_active(int d)
{
    pthread_mutex_lock(&g_bulk_lock);
    g_bulk_active += d;
    pthread_mutex_unlock(&g_bulk_lock);
}

static int bulk_read(int sock, word_t a_addr, unsigned long a_bytes, unsigned cap_mbps,
                     char *buf, size_t buf_len)
{
    unsigned rate, unpaced = ~0U;
    int ret = 0;
    static volatile uint8_t *win = NULL;
    char hdr[8];
    unsigned long off = (unsigned long)a_addr - MEM_WIN_BASE;
    int ok = off < MEM_WIN_SIZE && a_bytes <= MEM_WIN_SIZE - off && (off & 15) == 0;

    if (ok) {
        static pthread_mutex_t win_lock = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_lock(&win_lock);           /* first use, from any client */
        if (!win) {
            int wfd = open("/dev/mem", O_RDONLY | O_SYNC);
            if (wfd >= 0) {
                void *m = mmap(0, MEM_WIN_SIZE, PROT_READ, MAP_SHARED, wfd, DMA_BUF_BASE);
                if (m != MAP_FAILED) win = (volatile uint8_t *)m;
                close(wfd);
            }
        }
        pthread_mutex_unlock(&win_lock);
        if (!win) ok = 0;
    }
    hdr[0] = 'b';
    hdr[1] = (char)cap_mbps;
    hdr[2] = ok ? (char)((a_bytes >> 10) & 0xff) : 0;
    hdr[3] = ok ? (char)((a_bytes >> 18) & 0xff) : 0;
    memcpy(hdr + 4, &a_addr, 4);
    if (send_all(sock, hdr, 8) < 0) return -1;
    if (!ok) return 0;

    /* chunks of 64 KB: ~1 ms each at the paced rate, so a stream's share
     * follows the others starting and ending */
    buf_len &= ~(size_t)15;
    if (buf_len > 65536) buf_len = 65536;
    bulk_active(1);
    rate = bulk_rate(cap_mbps);
    setsockopt(sock, SOL_SOCKET, SO_MAX_PACING_RATE, &rate, sizeof(rate));
    while (a_bytes) {
        size_t n = a_bytes < buf_len ? a_bytes : buf_len, i;
        const volatile uint8_t *src = win + off;
#if defined(__aarch64__)
        for (i = 0; i < n; i += 16) {
            uint64_t lo, hi;
            __asm__ volatile("ldp %0, %1, [%2]" : "=&r"(lo), "=&r"(hi) : "r"(src + i) : "memory");
            memcpy(buf + i, &lo, 8);
            memcpy(buf + i + 8, &hi, 8);
        }
#else
        for (i = 0; i < n; i += 8) {
            uint64_t v = *(const volatile uint64_t *)(src + i);
            memcpy(buf + i, &v, 8);
        }
#endif
        if (send_all(sock, buf, n) < 0) { ret = -1; break; }
        off += n;
        a_bytes -= n;
        if (bulk_rate(cap_mbps) != rate) {
            rate = bulk_rate(cap_mbps);
            setsockopt(sock, SOL_SOCKET, SO_MAX_PACING_RATE, &rate, sizeof(rate));
        }
    }
    bulk_active(-1);
    setsockopt(sock, SOL_SOCKET, SO_MAX_PACING_RATE, &unpaced, sizeof(unpaced));
    return ret;
}
