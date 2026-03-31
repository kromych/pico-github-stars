MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

SECTIONS {
    .boot_info : ALIGN(4)
    {
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

/* move .text to start /after/ the boot info */
_stext = ADDR(.boot_info) + SIZEOF(.boot_info);

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
    *
    * Picotool looks through this block (as we have pointers to it in our
    * header) to find interesting information.
    */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    .flash_end : {
        __flash_binary_end = .;
    } > FLASH
} INSERT AFTER .uninit;
