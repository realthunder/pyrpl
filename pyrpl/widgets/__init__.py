"""Small helpers shared by the widgets."""


def escape_mnemonics(text):
    """Make `text` display literally in a widget that reads its text as
    mnemonic markup.

    Qt treats a single '&' as "the next character is the Alt shortcut": it
    swallows the ampersand and underlines that character, so a group title
    like 'MEMS circle & band' comes out as 'MEMS circle _band' (plus a stray
    accelerator). Measured on this Qt build, EVERY text-bearing widget used
    here does it - QPushButton, QToolButton, QCheckBox, QGroupBox titles,
    QTabWidget tabs, and QLabel even with no buddy - so any text that can
    carry a user's ampersand (a profile or config name, a device status
    string, a group title) goes through here on its way to setText/setTitle.

    Doubling the ampersand is the documented escape.
    """
    return str(text).replace('&', '&&')
