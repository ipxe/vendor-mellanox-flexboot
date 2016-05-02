/* At entry, the processor is in 16 bit real mode and the code is being
 * executed from an address it was not linked to. Code must be pic and
 * 32 bit sensitive until things are fixed up.
 *
 * Also be very careful as the stack is at the rear end of the interrupt
 * table so using a noticeable amount of stack space is a no-no.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL )

#include <config/general.h>
#include <config/branding.h>

#define PNP_SIGNATURE ( '$' + ( 'P' << 8 ) + ( 'n' << 16 ) + ( 'P' << 24 ) )
#define PMM_SIGNATURE ( '$' + ( 'P' << 8 ) + ( 'M' << 16 ) + ( 'M' << 24 ) )
#define PCI_SIGNATURE ( 'P' + ( 'C' << 8 ) + ( 'I' << 16 ) + ( ' ' << 24 ) )
#define STACK_MAGIC ( 'L' + ( 'R' << 8 ) + ( 'E' << 16 ) + ( 'T' << 24 ) )
#define PMM_ALLOCATE 0x0000
#define PMM_FIND 0x0001
#define PMM_HANDLE_BASE ( ( ( 'F' - 'A' + 1 ) << 26 ) + \
			  ( ( 'E' - 'A' + 1 ) << 21 ) + \
			  ( ( 'N' - 'A' + 1 ) << 16 ) )
#define PMM_HANDLE_BASE_IMAGE_SOURCE \
	( PMM_HANDLE_BASE | 0x00001000 )
#define PMM_HANDLE_BASE_DECOMPRESS_TO \
	( PMM_HANDLE_BASE | 0x00002000 )

/* ROM banner timeout, converted to a number of (18Hz) timer ticks. */
#define ROM_BANNER_TIMEOUT_TICKS ( ( 18 * ROM_BANNER_TIMEOUT ) / 10 )

/* Allow payload to be excluded from ROM size
 */
#if ROMPREFIX_EXCLUDE_PAYLOAD
#define	ZINFO_TYPE_ADxB "ADHB"
#define	ZINFO_TYPE_ADxW "ADHW"
#else
#define	ZINFO_TYPE_ADxB "ADDB"
#define	ZINFO_TYPE_ADxW "ADDW"
#endif

/* Allow ROM to be marked as containing multiple images
 */
#if ROMPREFIX_MORE_IMAGES
#define INDICATOR 0x00
#else
#define INDICATOR 0x80
#endif

/* Default to building a PCI ROM if no bus type is specified
 */
#ifndef BUSTYPE
#define BUSTYPE "PCIR"
#endif

	.text
	.code16
	.arch i386
	.section ".prefix", "ax", @progbits
	.globl	_rom_start
_rom_start:
	
	.org	0x00
romheader:
	.word	0xAA55			/* BIOS extension signature */
romheader_size:	.byte 0			/* Size in 512-byte blocks */
	jmp	init			/* Initialisation vector */
checksum:
	.byte	0
	.org	0x10
	.word	ipxeheader
	.org	0x16
	.word	undiheader
.ifeqs	BUSTYPE, "PCIR"
	.org	0x18
	.word	pciheader
.endif
	.org	0x1a
	.word	pnpheader
	.size romheader, . - romheader

	.section ".zinfo.fixup", "a", @progbits	/* Compressor fixups */
	.ascii	ZINFO_TYPE_ADxB
	.long	romheader_size
	.long	512
	.long	0
	.previous

.ifeqs	BUSTYPE, "PCIR"
pciheader:
	.ascii	"PCIR"			/* Signature */
	.word	pci_vendor_id		/* Vendor identification */ 
	.word	pci_device_id		/* Device identification */
	.word	( pci_devlist - pciheader ) /* Device list pointer */
	.word	pciheader_len		/* PCI data structure length */
	.byte	0x03			/* PCI data structure revision */
	.byte	0x02, 0x00, 0x00	/* Class code */
pciheader_image_length:
	.word	0			/* Image length */
	.word	0x0001			/* Revision level */
	.byte	0x00			/* Code type */
	.byte	INDICATOR		/* Last image indicator */
pciheader_runtime_length:
	.word	0			/* Maximum run-time image length */
	.word	0x0000			/* Configuration utility code header */
	.word	0x0000			/* DMTF CLP entry point */
	.equ pciheader_len, . - pciheader
	.size pciheader, . - pciheader

	/* PCI additional device list (filled in by linker) */
	.section ".pci_devlist.00000000", "a", @progbits
pci_devlist:
	.previous
	.section ".pci_devlist.ffffffff", "a", @progbits
pci_devlist_end:
	.short	0x0000 /* List terminator */
	.previous
	/* Ensure that terminator is always present */
	.reloc pciheader, RELOC_TYPE_NONE, pci_devlist_end

	.section ".zinfo.fixup", "a", @progbits	/* Compressor fixups */
	.ascii	ZINFO_TYPE_ADxW
	.long	pciheader_image_length
	.long	512
	.long	0
	.ascii	ZINFO_TYPE_ADxW
	.long	pciheader_runtime_length
	.long	512
	.long	0
	.previous
.endif	/* PCIR */

	/* PnP doesn't require any particular alignment, but IBM
	 * BIOSes will scan on 16-byte boundaries rather than using
	 * the offset stored at 0x1a
	 */
	.align	16
pnpheader:
	.ascii	"$PnP"			/* Signature */
	.byte	0x01			/* Structure revision */
	.byte	( pnpheader_len	/ 16 )	/* Length (in 16 byte increments) */
	.word	0x0000			/* Offset of next header */
	.byte	0x00			/* Reserved */
	.byte	0x00			/* Checksum */
	.long	0x00000000		/* Device identifier */
	.word	mfgstr			/* Manufacturer string */
	.word	prodstr			/* Product name */
	.byte	0x02			/* Device base type code */
	.byte	0x00			/* Device sub-type code */
	.byte	0x00			/* Device interface type code */
	.byte	0xf4			/* Device indicator */
	.word	0x0000			/* Boot connection vector */
	.word	0x0000			/* Disconnect vector */
	.word	bev_entry		/* Boot execution vector */
	.word	0x0000			/* Reserved */
	.word	0x0000			/* Static resource information vector*/
	.equ pnpheader_len, . - pnpheader
	.size pnpheader, . - pnpheader

/* Manufacturer string */
mfgstr:
	.asciz	PRODUCT_URI
	.size mfgstr, . - mfgstr

/* Product string
 *
 * Defaults to PRODUCT_SHORT_NAME.  If the ROM image is writable at
 * initialisation time, it will be filled in to include the PCI
 * bus:dev.fn number of the card as well.
 */
prodstr:
	.ascii	PRODUCT_NAME
.ifeqs	BUSTYPE, "PCIR"
prodstr_separator:
	.byte	0
	.ascii	"(PCI "
prodstr_pci_id:
	.ascii	"xx:xx.x)"		/* Filled in by init code */
.endif	/* PCIR */
	.byte	0
	.size prodstr, . - prodstr

	.globl	undiheader	
	.weak	undiloader
undiheader:
	.ascii	"UNDI"			/* Signature */
	.byte	undiheader_len		/* Length of structure */
	.byte	0			/* Checksum */
	.byte	0			/* Structure revision */
	.byte	0,1,2			/* PXE version: 2.1.0 */
	.word	undiloader		/* Offset to loader routine */
	.word	_data16_memsz		/* Stack segment size */
	.word	_data16_memsz		/* Data segment size */
	.word	_text16_memsz		/* Code segment size */
	.ascii	BUSTYPE			/* Bus type */
	.equ undiheader_len, . - undiheader
	.size undiheader, . - undiheader

ipxeheader:
	.ascii	"iPXE"			/* Signature */
	.byte	ipxeheader_len		/* Length of structure */
	.byte	0			/* Checksum */
shrunk_rom_size:
	.byte	0			/* Shrunk size (in 512-byte blocks) */
	.byte	0			/* Reserved */
build_id:
	.long	_build_id		/* Randomly-generated build ID */
	.equ ipxeheader_len, . - ipxeheader
	.size ipxeheader, . - ipxeheader

	.section ".zinfo.fixup", "a", @progbits	/* Compressor fixups */
	.ascii	"ADHB"
	.long	shrunk_rom_size
	.long	512
	.long	0
	.previous

/* Initialisation (called once during POST)
 *
 * Determine whether or not this is a PnP system via a signature
 * check.  If it is PnP, return to the PnP BIOS indicating that we are
 * a boot-capable device; the BIOS will call our boot execution vector
 * if it wants to boot us.  If it is not PnP, hook INT 19.
 */
init:
	/* Preserve registers, clear direction flag, set %ds=%cs */
	pushaw
	pushw	%ds
	pushw	%es
	pushw	%fs
	pushw	%gs
	cld
	pushw	%cs
	popw	%ds

	/* Print message as early as possible */
	movw	$init_message, %si
	xorw	%di, %di
	call	print_message

	/* Store PCI 3.0 runtime segment address for later use, if
	 * applicable.
	 */
.ifeqs	BUSTYPE, "PCIR"
	movw	%bx, %gs
.endif

	/* Store PCI bus:dev.fn address, print PCI bus:dev.fn, and add
	 * PCI bus:dev.fn to product name string, if applicable.
	 */
.ifeqs	BUSTYPE, "PCIR"
	xorw	%di, %di
	call	print_space
	movw	%ax, init_pci_busdevfn
	call	print_pci_busdevfn
	movw	$prodstr_pci_id, %di
	call	print_pci_busdevfn
	movb	$( ' ' ), prodstr_separator
.endif

	/* Print segment address */
	xorw	%di, %di
	call	print_space
	movw	%cs, %ax
	call	print_hex_word

	/* Check for PCI BIOS version, if applicable */
.ifeqs	BUSTYPE, "PCIR"
	pushl	%ebx
	pushl	%edx
	pushl	%edi
	stc
	movw	$0xb101, %ax
	int	$0x1a
	jc	no_pci3
	cmpl	$PCI_SIGNATURE, %edx
	jne	no_pci3
	testb	%ah, %ah
	jnz	no_pci3
	movw	$init_message_pci, %si
	xorw	%di, %di
	call	print_message
	movb	%bh, %al
	call	print_hex_nibble
	movb	$( '.' ), %al
	call	print_character
	movb	%bl, %al
	call	print_hex_byte
	cmpb	$3, %bh
	jb	no_pci3
	/* PCI >=3.0: leave %gs as-is if sane */
	movw	%gs, %ax
	cmpw	$0xa000, %ax	/* Insane if %gs < 0xa000 */
	jb	pci3_insane
	movw	%cs, %bx	/* Sane if %cs == %gs */
	cmpw	%bx, %ax
	je	1f
	movzbw	romheader_size, %cx /* Sane if %cs+len <= %gs */
	shlw	$5, %cx
	addw	%cx, %bx
	cmpw	%bx, %ax
	jae	1f
	movw	%cs, %bx	/* Sane if %gs+len <= %cs */
	addw	%cx, %ax
	cmpw	%bx, %ax
	jbe	1f
pci3_insane: /* PCI 3.0 with insane %gs value: print error and ignore %gs */
	movb	$( '!' ), %al
	call	print_character
	movw	%gs, %ax
	call	print_hex_word
no_pci3:
	/* PCI <3.0: set %gs (runtime segment) = %cs (init-time segment) */
	pushw	%cs
	popw	%gs
1:	popl	%edi
	popl	%edx
	popl	%ebx
.endif	/* PCIR */

	/* Check for PnP BIOS.  Although %es:di should point to the
	 * PnP BIOS signature on entry, some BIOSes fail to do this.
	 */
	movw	$( 0xf000 - 1 ), %bx
pnp_scan:
	incw	%bx
	jz	no_pnp
	movw	%bx, %es
	cmpl	$PNP_SIGNATURE, %es:0
	jne	pnp_scan
	xorw	%dx, %dx
	xorw	%si, %si
	movzbw	%es:5, %cx
1:	es lodsb
	addb	%al, %dl
	loop	1b
	jnz	pnp_scan
	/* Is PnP: print PnP message */
	movw	$init_message_pnp, %si
	xorw	%di, %di
	call	print_message
	jmp	pnp_done
no_pnp:	/* Not PnP-compliant - hook INT 19 */
#ifdef NONPNP_HOOK_INT19
	movw	$init_message_int19, %si
	xorw	%di, %di
	call	print_message
	xorw	%ax, %ax
	movw	%ax, %es
	pushl	%es:( 0x19 * 4 )
	popl	orig_int19
	pushw	%gs /* %gs contains runtime %cs */
	pushw	$int19_entry
	popl	%es:( 0x19 * 4 )
#endif /* NONPNP_HOOK_INT19 */
pnp_done:

	/* Check for PMM */
	movw	$( 0xe000 - 1 ), %bx
pmm_scan:
	incw	%bx
	jz	no_pmm
	movw	%bx, %es
	cmpl	$PMM_SIGNATURE, %es:0
	jne	pmm_scan
	xorw	%dx, %dx
	xorw	%si, %si
	movzbw	%es:5, %cx
1:	es lodsb
	addb	%al, %dl
	loop	1b
	jnz	pmm_scan
	/* PMM found: print PMM message */
	movw	$init_message_pmm, %si
	xorw	%di, %di
	call	print_message
	/* We have PMM and so a 1kB stack: preserve whole registers */
	pushal
	/* Allocate image source PMM block.  Round up the size to the
	 * nearest 4kB (8 512-byte sectors) to work around AMI BIOS bugs.
	 */
	movzbl	romheader_size, %ecx
	addw	extra_size, %cx
	addw	$0x0007, %cx	/* Round up to multiple of 8 512-byte sectors */
	andw	$0xfff8, %cx
	shll	$5, %ecx
	movl	$PMM_HANDLE_BASE_IMAGE_SOURCE, %ebx
	movw	$get_pmm_image_source, %bp
	call	get_pmm
	movl	%esi, image_source
	jz	1f
	/* Copy ROM to image source PMM block */
	pushw	%es
	xorw	%ax, %ax
	movw	%ax, %es
	movl	%esi, %edi
	xorl	%esi, %esi
	movzbl	romheader_size, %ecx
	shll	$7, %ecx
	addr32 rep movsl	/* PMM presence implies flat real mode */
	popw	%es
	/* Shrink ROM */
	movb	shrunk_rom_size, %al
	movb	%al, romheader_size
1:	/* Allocate decompression PMM block.  Round up the size to the
	 * nearest 128kB and use the size within the PMM handle; this
	 * allows the same decompression area to be shared between
	 * multiple iPXE ROMs even with differing build IDs
	 */
	movl	$_textdata_memsz_pgh, %ecx
	addl	$0x00001fff, %ecx
	andl	$0xffffe000, %ecx
	movl	%ecx, %ebx
	shrw	$12, %bx
	orl		$PMM_HANDLE_BASE_DECOMPRESS_TO, %ebx
	movw	$get_pmm_decompress_to, %bp
	call	get_pmm
	movl	%esi, decompress_to
	/* Restore registers */
	popal
no_pmm:

	/* Update checksum */
	xorw	%bx, %bx
	xorw	%si, %si
	movzbw	romheader_size, %cx
	shlw	$9, %cx
1:	lodsb
	addb	%al, %bl
	loop	1b
	subb	%bl, checksum

#ifdef FLASH_CONFIGURATION
	/* call for a procedure that will check device NVMEM configuration
	 * for BEV deletion this procedure may nullify BEV in the PnP header
	 */
	/* add init dev function code */

	pushw	%bx
	pushw   init_pci_busdevfn
	xorl	%ebx, %ebx
	popw	%bx
#ifdef DEVICE_CX3
	/*
	 * reading ppf to check if there is driver up
	 * having one indicates that this prefix is running while UEFI driver is up
	 * therefore we should not do any HCA commands, and gently continue (and posting BEV)
	 */
	call	test_device_ownership
	testl	%eax, %eax
	jnz	93f
	call 	check_nv_cfg_en
	call	get_flexboot_menu_to
#endif /* DEVICE_CX3 */
#ifndef DEVICE_CX3
	pushw	%cs
	call 	gw_get_cap_ofst
	cmpb	$0 ,mlx_cap_ofst
   	/* on error skip other functions */
    je		93f
#endif
#ifdef DEVICE_CX4
    call 	wait_fw_up
    testl	%eax, %eax
    jnz		93f
	call 	check_nv_cfg_en
	call	get_flexboot_menu_to
#endif /* DEVICE_CX4 */
#if defined ( DEVICE_CX4 ) || defined ( DEVICE_CX3 )
	/* push cs to make a 32 bit jump */
	pushw	%cs
	call	check_op_rom_en
#endif /* DEVICE_CX4 || DEVICE_CX3 */
93:
   popw		%bx
#endif /* FLASH_CONFIGURATION */
	/* Copy self to option ROM space, if applicable.  Required for
	 * PCI3.0, which loads us to a temporary location in low
	 * memory.  Will be a no-op for lower PCI versions.
	 */
.ifeqs	BUSTYPE, "PCIR"
	xorw	%di, %di
	call	print_space
	movw	%gs, %ax
	call	print_hex_word
	movzbw	romheader_size, %cx
	shlw	$9, %cx
	movw	%ax, %es
	xorw	%si, %si
	xorw	%di, %di
	cs rep	movsb
.endif
	/* check reasult and act according to it */
	cmpw 	$0x0, (flexboot_menu_to)
	jz		no_shell
	/* Prompt for POST-time shell */
	movw	$init_message_prompt, %si
	xorw	%di, %di
	call	print_message
	movw	$prodstr, %si
	call	print_message
	movw	$init_message_dots, %si
	call	print_message
	/* Wait for Ctrl-B */
	movw	$0xff02, %bx
	call	wait_for_key
	/* Clear prompt */
	pushf
	xorw	%di, %di
	call	print_kill_line
	movw	$init_message_done, %si
	call	print_message
	popf
	jnz	no_shell
	/* Ctrl-B was pressed: invoke iPXE.  The keypress will be
	 * picked up by the initial shell prompt, and we will drop
	 * into a shell.
	 */
	movw	$0x0001, init_post_shell
	xorl	%ebp, %ebp	/* Inhibit use of INT 15,e820 and INT 15,e801 */
	pushw	%cs
	call	exec
no_shell:
	movb	$( '\n' ), %al
	xorw	%di, %di
	call	print_character

	/* Restore registers */
	popw	%gs
	popw	%fs
	popw	%es
	popw	%ds
	popaw

	/* Indicate boot capability to PnP BIOS, if present */
	movw	$0x20, %ax
	lret
	.size init, . - init

/* Attempt to find or allocate PMM block
 *
 * Parameters:
 *  %ecx : size of block to allocate, in paragraphs
 *  %ebx : PMM handle base
 *  %bp : routine to check acceptability of found blocks
 *  %es:0000 : PMM structure
 * Returns:
 *  %ebx : PMM handle
 *  %esi : allocated block address, or zero (with ZF set) if allocation failed
 */
get_pmm:
	/* Preserve registers */
	pushl	%eax
	pushw	%di
	movw	$( ' ' ), %di
get_pmm_find:
	/* Try to find existing block */
	pushl	%ebx		/* PMM handle */
	pushw	$PMM_FIND
	lcall	*%es:7
	addw	$6, %sp
	pushw	%dx
	pushw	%ax
	popl	%esi
	/* Treat 0xffffffff (not supported) as 0x00000000 (not found) */
	incl	%esi
	jz	get_pmm_allocate
	decl	%esi
	jz	get_pmm_allocate
	/* Block found - check acceptability */
	call	*%bp
	jnc	get_pmm_done
	/* Block not acceptable - increment handle and retry */
	incl	%ebx
	jmp	get_pmm_find
get_pmm_allocate:
	/* Block not found - try to allocate new block */
	pushw	$0x0002		/* Extended memory */
	pushl	%ebx		/* PMM handle */
	pushl	%ecx		/* Length */
	pushw	$PMM_ALLOCATE
	lcall	*%es:7
	addw	$12, %sp
	pushw	%dx
	pushw	%ax
	popl	%esi
	movw	$( '+' ), %di	/* Indicate allocation attempt */
get_pmm_done:
	/* Print block address */
	movw	%di, %ax
	xorw	%di, %di
	call	print_character
	movl	%esi, %eax
	call	print_hex_dword
	/* Treat 0xffffffff (not supported) as 0x00000000 (allocation
	 * failed), and set ZF to indicate a zero result.
	 */
	incl	%esi
	jz	1f
	decl	%esi
1:	/* Restore registers and return */
	popw	%di
	popl	%eax
	ret
	.size	get_pmm, . - get_pmm

	/* Check acceptability of image source block */
get_pmm_image_source:
	pushw	%es
	xorw	%ax, %ax
	movw	%ax, %es
	movl	build_id, %eax
	addr32 cmpl %es:build_id(%esi), %eax
	je	1f
	stc
1:	popw	%es
	ret
	.size	get_pmm_image_source, . - get_pmm_image_source

	/* Check acceptability of decompression block */
get_pmm_decompress_to:
	clc
	ret
	.size	get_pmm_decompress_to, . - get_pmm_decompress_to

/*
 * Note to hardware vendors:
 *
 * If you wish to brand this boot ROM, please do so by defining the
 * strings PRODUCT_NAME and PRODUCT_SHORT_NAME in config/branding.h.
 *
 * While nothing in the GPL prevents you from removing all references
 * to iPXE or http://ipxe.org, we prefer you not to do so.
 *
 * If you have an OEM-mandated branding requirement that cannot be
 * satisfied simply by defining PRODUCT_NAME and PRODUCT_SHORT_NAME,
 * please contact us.
 *
 * [ Including an ASCII NUL in PRODUCT_NAME is considered to be
 *   bypassing the spirit of this request! ]
 */
init_message:
	.ascii	"\n"
	.ascii	PRODUCT_NAME
	.ascii	"\n"
	.ascii	PRODUCT_SHORT_NAME
	.ascii	" ("
	.ascii	PRODUCT_URI
	.asciz	")"
	.size	init_message, . - init_message
.ifeqs	BUSTYPE, "PCIR"
init_message_pci:
	.asciz	" PCI"
	.size	init_message_pci, . - init_message_pci
.endif	/* PCIR */
init_message_pnp:
	.asciz	" PnP"
	.size	init_message_pnp, . - init_message_pnp
init_message_pmm:
	.asciz	" PMM"
	.size	init_message_pmm, . - init_message_pmm
init_message_int19:
	.asciz	" INT19"
	.size	init_message_int19, . - init_message_int19
init_message_prompt:
	.asciz	"\nPress Ctrl-B to configure "
	.size	init_message_prompt, . - init_message_prompt
init_message_dots:
	.asciz	"..."
	.size	init_message_dots, . - init_message_dots
init_message_done:
	.asciz	"\n\n"
	.size	init_message_done, . - init_message_done

init_post_shell:
	.word   0
	.size   init_post_shell, . - init_post_shell

flexboot_menu_to:
	.word 0
	.size flexboot_menu_to, . - flexboot_menu_to

#ifndef DEVICE_CX3
mlx_cap_ofst:
	.byte 0
	.size mlx_cap_ofst, . - mlx_cap_ofst
#endif
/* if this flag is 0 then can not read values from TLV and will use only ini */
nv_cfg_en_f:
	.byte 0
	.size nv_cfg_en_f, . - nv_cfg_en_f

/* PCI bus:dev.fn
 *
 */
.ifeqs	BUSTYPE, "PCIR"
init_pci_busdevfn:
	.word	0
	.size	init_pci_busdevfn, . - init_pci_busdevfn
.endif	/* PCIR */

/* Image source area
 *
 * May be either zero (indicating to use option ROM space as source),
 * or within a PMM-allocated block.
 */
	.globl	image_source
image_source:
	.long	0
	.size	image_source, . - image_source

/* Additional image source size (in 512-byte sectors)
 *
 */
extra_size:
	.word	0
	.size	extra_size, . - extra_size

/* Temporary decompression area
 *
 * May be either zero (indicating to use default decompression area in
 * high memory), or within a PMM-allocated block.
 */
	.globl	decompress_to
decompress_to:
	.long	0
	.size	decompress_to, . - decompress_to

/* Boot Execution Vector entry point
 *
 * Called by the PnP BIOS when it wants to boot us.
 */
bev_entry:
	pushl	%esi
	pushl	%ebx
	pushw   %ds
	pushw   %es
	orl	$0xffffffff, %ebp	/* Allow arbitrary relocation */
	pushw	%cs
	call	exec
	popw	%es
	popw	%ds
	popl	%ebx
	popl	%esi
	lret
	.size	bev_entry, . - bev_entry

/* INT19 entry point
 *
 * Called via the hooked INT 19 if we detected a non-PnP BIOS.  We
 * attempt to return via the original INT 19 vector (if we were able
 * to store it).
 */
int19_entry:
	pushw	%cs
	popw	%ds
	/* Prompt user to press B to boot */
	movw	$int19_message_prompt, %si
	xorw	%di, %di
	call	print_message
	movw	$prodstr, %si
	call	print_message
	movw	$int19_message_dots, %si
	call	print_message
	movw	$0xdf4e, %bx
	call	wait_for_key
	pushf
	xorw	%di, %di
	call	print_kill_line
	movw	$int19_message_done, %si
	call	print_message
	popf
	jz	1f
	/* Leave keypress in buffer and start iPXE.  The keypress will
	 * cause the usual initial Ctrl-B prompt to be skipped.
	 */
	orl	$0xffffffff, %ebp	/* Allow arbitrary relocation */
	pushw	%cs
	call	exec
1:	/* Try to call original INT 19 vector */
	movl	%cs:orig_int19, %eax
	testl	%eax, %eax
	je	2f
	ljmp	*%cs:orig_int19
2:	/* No chained vector: issue INT 18 as a last resort */
	int	$0x18
	.size	int19_entry, . - int19_entry
orig_int19:
	.long	0
	.size	orig_int19, . - orig_int19

int19_message_prompt:
	.asciz	"Press N to skip booting from "
	.size	int19_message_prompt, . - int19_message_prompt
int19_message_dots:
	.asciz	"..."
	.size	int19_message_dots, . - int19_message_dots
int19_message_done:
	.asciz	"\n\n"
	.size	int19_message_done, . - int19_message_done
	
/* Execute as a boot device
 *
 */
exec:	/* Set %ds = %cs */
	pushw	%cs
	popw	%ds

	/* Print message as soon as possible */
	movw	$prodstr, %si
	xorw	%di, %di
	call	print_message
	movw	$exec_message_pre_install, %si
	call	print_message

	/* Store magic word on BIOS stack and remember BIOS %ss:sp */
	pushl	$STACK_MAGIC
	movw	%ss, %cx
	movw	%sp, %dx

	/* Obtain a reasonably-sized temporary stack */
	xorw	%bx, %bx
	movw	%bx, %ss
	movw	$0x7c00, %sp

	/* Install iPXE */
	call	alloc_basemem
	movl	image_source, %esi
	movl	decompress_to, %edi
	call	install_prealloc

	/* Copy pci_busdevfn and post_shell to .data16 */
	pushw   %es
	movw    %bx, %es
	pushw   init_pci_busdevfn
	popw    %es:boot_pci_busdevfn
	pushw	init_post_shell
	cmpw	$0x0000, init_post_shell
	jz	skip_clear_init_post_shell
	movw	$0x0000, init_post_shell
skip_clear_init_post_shell:
	popw	%es:boot_post_shell
	popw    %es

	/* Print message indicating successful installation */
	movw	$exec_message_post_install, %si
	xorw	%di, %di
	call	print_message

	/* Set up real-mode stack */
	movw	%bx, %ss
	movw	$_estack16, %sp

	/* Jump to .text16 segment */
	pushw	%ax
	pushw	$1f
	lret
	.section ".text16", "awx", @progbits
1:
	/* Retrieve PCI bus:dev.fn, if applicable */
.ifeqs	BUSTYPE, "PCIR"
	movw	init_pci_busdevfn, %ax
.endif

	/* Set up %ds for access to .data16 */
	movw	%bx, %ds

	/* Store PCI bus:dev.fn, if applicable */
.ifeqs	BUSTYPE, "PCIR"
#ifdef AUTOBOOT_ROM_FILTER
	movw	%ax, autoboot_busdevfn
#endif /* AUTOBOOT_ROM_FILTER */
.endif

	/* Call main() */
	pushl	$main
	pushw	%cs
	call	prot_call
	popl	%eax /* discard */

	/* Set up flat real mode for return to BIOS */
	call	flatten_real_mode

	/* Uninstall iPXE */
	call	uninstall

	/* Restore BIOS stack */
	movw	%cx, %ss
	movw	%dx, %sp

	/* Check magic word on BIOS stack */
	popl	%eax
	cmpl	$STACK_MAGIC, %eax
	jne	1f
	/* BIOS stack OK: return to caller */
	lret
1:	/* BIOS stack corrupt: use INT 18 */
	int	$0x18
	.previous

exec_message_pre_install:
	.asciz	" starting execution..."
	.size exec_message_pre_install, . - exec_message_pre_install
exec_message_post_install:
	.asciz	"ok\n"
	.size exec_message_post_install, . - exec_message_post_install

#ifdef FLASH_CONFIGURATION
/*
 * Description - check device NVMEM configuration for BEV deletion
 * IN  	       -  NON
 * OUT 	       -  %eax:  0 iff BEV nullified
 */
check_op_rom_en:
	pushl 	%esi
#ifdef DEVICE_CX3
	movl    $1, %eax
	/* get number of ports , store in %esi */
	call	get_num_ports
 	cmpl    $0xffffffff, %eax
 	je 	delete_bev

 	/* get port conf - start from port 1 */
	movl	%eax, %esi;
	movl 	$1, %eax
port_loop:
	pushl	%eax
	call 	check_port_en
	testl	%eax, %eax
	popl	%eax
	jnz	99f
	incl	%eax
	decl	%esi
	jnz	port_loop
#elif defined(DEVICE_CX4)
	call	check_host_op_rom_en
	testl	%eax, %eax
	jnz	99f
#endif
delete_bev:
	movl	$0x0, %eax
	movb    $0x0, (pnpheader + 0x1a)
	movb    $0x0, (pnpheader + 0x1b)
99:
	popl	%esi
	lret
	.size check_op_rom_en, . - check_op_rom_en
#endif /* FLASH_CONFIGURATION */

/* Wait for key press specified by %bl (masked by %bh)
 *
 * Used by init and INT19 code when prompting user.  If the specified
 * key is pressed, it is left in the keyboard buffer.
 *
 * Returns with ZF set iff specified key is pressed.
 */
wait_for_key:
	/* Preserve registers */
	pushw	%cx
	pushw	%ax
1:	/* Empty the keyboard buffer before waiting for input */
	movb	$0x01, %ah
	int		$0x16
	jz		2f
	xorw	%ax, %ax
	int		$0x16
	jmp		1b
2:	/* Wait for a key press */
	movw	(flexboot_menu_to), %cx
3:	decw	%cx
	js		99f		/* Exit with ZF clear */
	/* Wait for timer tick to be updated */
	call	wait_for_tick
	/* Check to see if a key was pressed */
	movb	$0x01, %ah
	int		$0x16
	jz		3b
	/* Check to see if key was the specified key */
	andb	%bh, %al
	cmpb	%al, %bl
	je		99f		/* Exit with ZF set */
	/* Not the specified key: remove from buffer and stop waiting */
	pushfw
	xorw	%ax, %ax
	int		$0x16
	popfw			/* Exit with ZF clear */
99:	/* Restore registers and return */
	popw	%ax
	popw	%cx
	ret
	.size wait_for_key, . - wait_for_key

/* Wait for timer tick
 *
 * Used by wait_for_key
 */
wait_for_tick:
	pushl	%eax
	pushw	%fs
	movw	$0x40, %ax
	movw	%ax, %fs
	movl	%fs:(0x6c), %eax
1:	pushf
	sti
	hlt
	popf
	cmpl	%fs:(0x6c), %eax
	je		1b
	popw	%fs
	popl	%eax
	ret
	.size wait_for_tick, . - wait_for_tick

#ifdef FLASH_CONFIGURATION

#define PCIBIOS_WRITE_CONFIG_WORD	0xb10c
#define PCIBIOS_WRITE_CONFIG_DWORD	0xb10d
#define PCIBIOS_READ_CONFIG_DWORD	0xb10a

/* Read PCI config dword
 *
 * Parameters:
 *   %bx : PCI bus:dev.fn
 *   %di : PCI register number
 * Returns:
 *   %ecx : Dword value
 */
my_pci_read_config_dword:
	/* Preserve registers */
	pushl	%eax
	pushl	%ebx
	pushl	%edx

	/* Issue INT 0x1a,b10a */
	movw	$PCIBIOS_READ_CONFIG_DWORD, %ax
	int	$0x1a

	/* Restore registers and return */
	popl	%edx
	popl	%ebx
	popl	%eax
	ret
	.size	my_pci_read_config_dword, . - my_pci_read_config_dword

/* Write PCI config dword
 *
 * Parameters:
 *   %bx : PCI bus:dev.fn
 *   %di : PCI register number
 *   %ecx : PCI BAR value
 * Returns:
 *   none
 */
my_pci_write_config_dword:
	/* Preserve registers */
	pushal

	/* Issue INT 0x1a,b10d */
	movw	$PCIBIOS_WRITE_CONFIG_DWORD, %ax
	int	$0x1a

	/* Restore registers and return */
	popal
	ret
	.size	my_pci_write_config_dword, . - my_pci_write_config_dword


#define TLV_ROM_BNR_ENTRY_OFST 0x2
#define INI_PARAM_ROM_BNR_MB_OFST 0x0
#define INI_PARAM_ROM_BNR_TO_BIT_OFST 16
#define DEFAULT_BANNER_TIMEOUT	14
#define ROM_BANNER_TO_TLV_MASK 0xf

#ifdef DEVICE_CX3
#define TLV_ROM_BNR_TYPE 0x2010
#define GW_ADDR_ADDR 0x58
#define GW_DATA_ADDR 0x5c
#define HCR_INPUT_H_OFFSET 0x80680
#define HCR_INPUT_L_OFFSET 0x80684

#define HCR_INPUT_MOD_OFFSET 0x80688
#define	HCR_OUTPUT_L_OFFSET 0x80690
#define	HCR_OUTPUT_H_OFFSET 0x8068C
#define HCR_TOKEN_MOD_OFFSET 0x80694
#define HCR_OFFSET 0x80698
#define HCR_PPF_OFFSET 0x1f724 
#define HCR_TOKEN	 0xffff0000
#define HCR_OPMOD_SHIFT 12
#define	HCR_T_BIT 21
#define HCR_GO_BIT 23
#define HCR_STATUS_BIT 24
#define HCR_GO_BIT_SET 0x800000
//
#define MOD_STAT_CFG_MAC_NUM_PORTS_OFFSET  0x40
#define MOD_STAT_CFG_MAC_L_WORD_OFFSET  0x94
#define MOD_STAT_CFG_GET_INLINE_OP_MOD  0x3
#define MOD_STAT_CFG_GET_INLINE_FLASH_ACCESS_MOD 0xE
#define MOD_STAT_CFG_GET_INLINE_MOD 		 0x3
#define MLX4_CMD_MOD_STAT_CFG		 	 0x34
#define MLX4_CMD_MB_READ			 0x71
#define MLX4_CMD_QUERY_DEF_PARAM 0x73
//
#define OPTION_ROM_EN_MB_OFFSET			 0x0
#define OPTION_ROM_INI_EN_BIT			 29
#define OPTION_ROM_TLV_EN_BIT			 31
#define OPTION_ROM_TLV_PROTOCOL_WORD	 16
//
#define CMD_TIMEOUT_IN_SEC	10
#define CMD_TIMEOUT_IN_TICKS ( 2 * ( 18 * CMD_TIMEOUT_IN_SEC ) / 10 )
//
#define MLX4_CMD_QUERY_DEV_CAP	0x3
#define NV_CNFG_EN_MB_OFFSET    0x0
#define NV_CNFG_EN_BIT			 23
/*
 * Description - polling GO bit till 10 secs t.o 
 * IN  	       - dont care
 * OUT 	       - cmd status - eax
 */
cmd_pend:
 	pushl	%esi
 	movl	$CMD_TIMEOUT_IN_TICKS, %esi
 1:
 	decl	%esi
 	js		99f /* Exit with ZF clear */
 	call	wait_for_tick
	/*test SW ownership*/
	movl	$HCR_OFFSET, %eax
	call 	gw_read_dword
	movl	%eax, %edx
	shrl	$HCR_GO_BIT, %eax
	andl	$0x1, %eax
	testw	%ax, %ax
	jnz		1b
	/*status - HCR*/
	shrl	$HCR_STATUS_BIT, %edx
	movl	%edx, %eax
	jmp		2f
	/*sw ownership - no need to pend anymore OR timeout*/
99:
	/*status - fail*/
	movl	$0xffffffff, %eax
2:
	popl 	%esi
	ret
 .size	cmd_pend, . - cmd_pend

test_device_ownership:
	pushl	%edi
	movw    $GW_ADDR_ADDR, %di
	movl	$HCR_PPF_OFFSET, %eax
	call 	gw_read_dword
	popl	%edi
	ret
.size  test_device_ownership, . - test_device_ownership


/*
 * Description - check device NVMEM configuration enabled
 * IN  	       -  NON
 * OUT 	       -  %eax:  1 if non-volatile config supported
 * side effects - the global parameter nv_cfg_en_f is updated according to eax
 */
check_nv_cfg_en:
	/* save edx register on stack */
	pushl	%edx
 	/* eax = 0 for global defaults as parameter */
 	movl	$0, %eax
 	/* place holder for the lower part of the 64 bit value returned. */
	pushl 	$0xffffffff
 	/* offset in Mailbox to read 64 bit from */
	pushl 	$NV_CNFG_EN_MB_OFFSET
 	/* ini cmd opcode */
 	pushl	$MLX4_CMD_QUERY_DEF_PARAM
	call 	read_ini
	/* pop passed parameters (data suppose to be on stack) */
	addl    $8 , %esp
	popl	%edx
	cmpl	$0xffffffff, %eax
	je		1f
	/* check default_sriov_en bit */
	shrl    $NV_CNFG_EN_BIT, %edx
	andl	$0x1, %edx
	cmpl	$0x1, %edx
	je		2f
1:
	movl	$0x0,	%eax
	movb	$0x0, nv_cfg_en_f
	jmp		3f
2:
	movl	$0x1,	%eax
	movb	$0x1, nv_cfg_en_f
3:
	/* pop previous edx value saved on stack */
	popl	%edx
	ret
.size check_nv_cfg_en, . - check_nv_cfg_en

/*
 * Description -  parse the resualt according to mask and flag.
 *
 *  IN
 *			   - esp:
 *					 (esp + 2)  = the 64 bit low dword
 *					 (esp + 6)  = the 64 bit high dword
 *					 (esp + 10) = IsInLowDword flag (if 0 then use upper dword)
 *			   		 (esp + 14) = MASK to read from
 *			   		 (esp + 18) = place holder for the resault
 *
 * OUT
 *			   - esp:
 *					 (esp + 18)= dword resault
 */

read_tlv_ini_res:

	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now paramters starts from (ebp + 6) */
	movl 	%esp,%ebp
	pushl	%eax
	pushl	%edx
	pushl 	%ecx
	/* check if need to work on low dword */
	cmpl	$0x1, %ss:14(%ebp)
	jne		1f
	/* copy low dword */
	movl	%ss:6(%ebp), %edx
	jmp		2f
1:
	/* copy high dword */
	movl	%ss:10(%ebp), %edx
2:
	/* copy mask */
	movl	%ss:18(%ebp), %eax

	/* apply mask */
	andl 	%eax, %edx

	/* shift resault */
shift_loop:
	movl	%eax,%ecx
	andl 	$0x01, %ecx
	jnz 	3f
	shrl 	$1,%eax
	shrl 	$1,%edx
	jmp 	shift_loop

3:
	movl 	%edx,%ss:22(%ebp)
	popl 	%ecx
	popl	%edx
	popl	%eax
	popl	%ebp
	ret

.size read_tlv_ini_res, . - read_tlv_ini_res

/*
 * Description -  this function is auxiliary function to make code clearer
 *				  at function read_tlv_ini_cfg_val and should be used only
 *				  by read_tlv_ini_cfg_val.
 */
read_tlv_ini_get_tlv:
/* read port num or 0 for global defaults
		and pass parameter to get_tlv_value */
	movl	%ss:6(%ebp), %eax
	/* place holder for the lower word of the returned QWORD */
	pushl 	$0xffffffff;
	/* push TLV entry offset */
	movl	%ss:10(%ebp), %edx
	pushl	%edx
	/* push setting type to the stack */
	movl	%ss:14(%ebp), %edx
	pushl	%edx
	/* read TLV value, returned is 64 bit %eax high word,[esp] + 6 lower word */
	call	get_tlv_value
	/* pop passed parameters */
	addl    $8 , %esp
	/* pop the lower dword of 64 bit value returned */
	popl	%edx
	/* get back out resault save it to %ecx */
	/* place holder for the 32 bit resault */
	pushl	$0xffffffff
	/* push TLV mask */
	movl	%ss:26(%ebp), %ecx
	pushl 	%ecx
	/* push Tlv flag */
	movl	%ss:30(%ebp), %ecx
	pushl 	%ecx
	/* push high dword */
	pushl	%eax
	/* push low dword  */
	pushl	%edx
	call 	read_tlv_ini_res
	/* pop passed parameters */
	addl    $16 , %esp
	/* pop resault */
	popl 	%ecx
	ret
.size read_tlv_ini_get_tlv, . - read_tlv_ini_get_tlv

/*
 * Description -  this function is auxiliary function to make code clearer
 *				  at function read_tlv_ini_cfg_val and should be used only
 *				  by read_tlv_ini_cfg_val.
 */
read_tlv_ini_read_ini:
	/* read port num or 0 for global defaults
		and pass parameter to read_ini */
	movl	%ss:6(%ebp), %eax
	/* place holder for the lower part of the 64 bit value returned. */
	pushl 	$0xffffffff
	/* offset inside the MB to read from. */
	movl	%ss:18(%ebp), %edx
	pushl	%edx
 	/* ini cmd opcode */
 	movl	%ss:22(%ebp), %edx
	pushl	%edx
	call 	read_ini
	/* pop passed parameters (%eax holds the wanted param) */
	addl    $8 , %esp
	/* pop the lower dword of 64 bit value returned */
	popl	%edx
	/* get back out resault save it to %ecx */
	/* place holder for the 32 bit resault */
	pushl	$0xFFFFFFFF
	/* push ini mask */
	movl	%ss:34(%ebp), %ecx
	pushl 	%ecx
	/* push ini flag */
	movl	%ss:38(%ebp), %ecx
	pushl 	%ecx
	/* push high dword */
	pushl	%eax
	/* push low dword  */
	pushl	%edx
	call 	read_tlv_ini_res
	/* pop passed parameters */
	addl    $16 , %esp
	/* pop resault */
	popl 	%ecx
	ret
.size read_tlv_ini_read_ini, . - read_tlv_ini_read_ini

/*
 * Description -  return qword of a TLV at givin type and entry offset if
 *				  failed will return qword ini configuration value at givin port
 *				  or default param (port = 0) and at givin mailbox offset,
 *				  on failure:
 *				  the high dword returned will be 0xFFFFFFFF (assuming this
 *				  is not legal value).
 *  IN
 *			   - esp:
 *					 (esp + 2)  = port num or 0 for global defaults
 *					 (esp + 6)  = TLV_ENTRY_OFFST
 *					 (esp + 10) = TLV_TYPE (if 0 then will skip TLV reading)
 *			   		 (esp + 14) = offset in Mailbox ini to read 64 bit from
 *			   		 (esp + 18) = opcode of the ini cmd (could be 0x72 or 0x73)
 *					 (esp + 22) = MASK for TLV
 *					 (esp + 26) = IsInLowDword TLV
 *					 (esp + 30) = MASK for ini
 *					 (esp + 34) = IsInLowDword  ini
 *			   		 (esp + 38) = default return value on error
 *			   		 (esp + 42) = place holder for the 64 bit low dword
 *			   		 (esp + 46) = place holder for the 64 bit high dword
 *					 (esp + 50) = place holder for  resault.
 * OUT
 *			   - esp:
 *					 (esp + 42)= low dword of the 64 bit value
 *			    	 (esp + 46) = high dword of 64 bit value (value of
 *								 0xFFFFFFFF endicates error).
 * 					 (esp + 50) = resault dword.
 *			   - eax:
 *					 eax = 0 Value was read from TLV
 *					 eax = 1 Value was read from INI
 *					 eax = ffffffff failure (default value returned as resault)
 *
 */
read_tlv_ini_cfg_val:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now parameters starts from (ebp + 6) */
	movl 	%esp,%ebp
	pushl	%ecx
	/* save edx register to stack */
	pushl 	%edx
	/* save esi register to stack (will be used as flag) */
	pushl	%esi

	/* if TLV configuration disabled will skip TLV and jump to read from ini. */
	cmpb	$0x0, (nv_cfg_en_f)
	je 		1f
	/* if TLV TYPE = 0 will skip TLV and jump to read from ini. */
	cmpl	$0x0, %ss:14(%ebp)
	je 		1f
	call 	read_tlv_ini_get_tlv
	/* value read from tlv */
	movl	$0, %esi
	/* test failure */
	cmpl	$0xffffffff, %eax
	jne 	2f /* no error - assuming no TLV contect of 0xfffffff */

1:
	/* no TLV - check INI default */
	call	read_tlv_ini_read_ini
	/* value read from ini */
	movl	$1, %esi
	cmpl	$0xffffffff, %eax
	jne 	2f /* no error - assuming no INI contect of 0xfffffff */
	/* error code assign %ecx default value */
	movl	%ss:42(%ebp), %ecx
	/* error code */
	movl	$0xffffffff, %esi
2:
	/* at this point %eax holds the upper dword, %edx hold the lower dword */
	/* put low dword on stack */
	movl	%edx, %ss:46(%ebp)
	/* put high dword on stack value of 0xFFFFFFFF indecates error */
	movl	%eax, %ss:50(%ebp)
	/* put returned value */
	movl 	%ecx, %ss:54(%ebp)
	/* esi return back flag value */
	movl	%esi, %eax
	/* pop registers previous values */
	popl	%esi
	popl	%edx
	popl	%ecx
	popl 	%ebp
	ret
.size read_tlv_ini_cfg_val, . - read_tlv_ini_cfg_val




/*
 * Description - set flexboot menu time out from setting in TLV, if not exist
 				 then using ini, otherwise will use default.
 * IN  	       -  NON
 * OUT		   -  NON
 * side effects - flexboot_menu_to global parameter is set
 */
get_flexboot_menu_to:
	/* save edx/eax register to stack */
	pushl	%eax
	pushl	%edx
	/* place holder for resault */
	pushl	$0x0
	/* high dword of 64 bit value */
	pushl	$0xffffffff
	/* low dword of 64 bit value */
	pushl	$0xffffffff
	/* default value in case of error */
	pushl	$DEFAULT_BANNER_TIMEOUT
	/* ini is_in_low_dword flag */
	pushl	$0x0
	/* ini resault mask */
	pushl	$0xF0000
	/* TLV is_in_low_dword */
	pushl	$0x0
	/* TLV resault mask */
	pushl	$ROM_BANNER_TO_TLV_MASK
	/* ini cmd opcode */
 	pushl	$MLX4_CMD_QUERY_DEF_PARAM
	/* offset inside the MB returned from ini to read from */
	pushl 	$INI_PARAM_ROM_BNR_MB_OFST
	/* push TLV type to the stack */
	pushl	$TLV_ROM_BNR_TYPE
	/* push TLV entry offset */
	pushl	$TLV_ROM_BNR_ENTRY_OFST
	/* push 0 for default global params */
	pushl	$0
	call 	read_tlv_ini_cfg_val
	/* pop passed parameter */
	addl    $48 , %esp
	/* pop resault */
	popl	%eax

	movl $0, %edx
	/* 18 * ax = (ax*16+ax*2) */
	movw	%ax, %dx
	shlw	$4, %ax
	shlw	$1, %dx
	addw	%dx, %ax
	movw	%ax,(flexboot_menu_to)
	/* pop registers previous values */
	popl	%edx
	popl	%eax
	ret
.size get_flexboot_menu_to, . - get_flexboot_menu_to

/*
* read TLV , no TLV read INI.
* if NO TLV & INI return dis (0)
*
* IN - eax : port number
* OUT - eax :  en(1) / dis (0)
*
*/
#define BOOT_SETTINGS_TYPE 0x2021
#define	OP_ROM_EN_INI_MASK 0x20000000
#define	OP_ROM_EN_TLV_MASK 0x80ff0000
#define	OP_ROM_EN_TLV_OFST 0x2
check_port_en:
	/* save edx/eax/ecx register to stack */
	pushl	%edx
	pushl	%ecx
	/* place holder for resault */
	pushl	$0x0
	/* place holder high dword of 64 bit value */
	pushl	$0xffffffff
	/* place holder low dword of 64 bit value */
	pushl	$0xffffffff
	/* default value in case of error */
	pushl	$0
	/* ini is_in_low_dword flag */
	pushl	$0x0
	/* ini resault mask */
	pushl	$OP_ROM_EN_INI_MASK
	/* TLV is_in_low_dword */
	pushl	$0x0
	/* TLV resault mask (legacy protocol + en bit) */
	pushl	$OP_ROM_EN_TLV_MASK
	/* ini cmd opcode */
 	pushl	$MLX4_CMD_QUERY_DEF_PARAM
	/* offset inside the MB returned from ini to read from */
	pushl 	$OPTION_ROM_EN_MB_OFFSET
	/* push TLV type to the stack */
	pushl	$BOOT_SETTINGS_TYPE
	/* push TLV entry offset */
	pushl	$OP_ROM_EN_TLV_OFST
	/* push port number */
	pushl	%eax
	call 	read_tlv_ini_cfg_val
	/* pop passed parameter */
	addl    $48 , %esp
	/* pop resault */
	popl	%ecx
	/* check failure */
	cmpl	$0xffffffff, %eax
	je		98f
	/* check if value was read from ini */
	cmpl	$0x1, %eax
	je		98f
	/* value was read from TLV */
	/* save option rom enable bit value */
	movl	%ecx, %edx
	/* legacy boot protocol check */
	andl	$0xff, %ecx
	jz		98f
   /* check enable option rom bit (use 15 insted of 31 cause already shifted) */
	shrl	$15, %edx
	movl	%edx, %ecx
	andl	$0x1, %ecx
98:
	movl 	%ecx, %eax
99:
	popl	%ecx
	popl	%edx
	ret
.size check_port_en, . -check_port_en 

/*
 * Description - Read INI , using ROM INI cmds (QUERY_ROMINI / QUERY_DEFPARAMS)
 				& read MB cmd
 *
 * IN  	       - eax : port num or 0 for global defaults
 *			   - esp: (esp + 2) = opcode of the ini cmd (could be 0x72 or 0x73)
 *			   - esp: (esp + 6) = offset in Mailbox to read 64 bit from
 * OUT 		   - eax : high dword of 64 bit from Mailbox
 *			   - esp :(esp + 10)	= lower dword of the 64 bit from Mailbox
 *
 */
read_ini:
	/* call rom ini */
	/* save ebp register on stack */
	pushl 	%ebp
	/* update ebp value (now paramters starts from (ebp + 6)) */
	movl 	%esp,%ebp
	/* save edx register on stack */
	pushl 	%edx;
	/* place holder for the lower part of the 64 bit value returned. */
	pushl 	$0xffffffff
	/* read opcode form stack from stack */
	movl	%ss:6(%ebp), %edx
	/* opcode */
	pushl 	%edx
	/* opcode modifier (port num or 0 for global defaults) */
	pushl	%eax
	/* input_mod  */
	pushl 	$0
	/* in_param_l */
	pushl 	$0 
	/* in_param_h */
	pushl 	$0	
	/* invoke command */
	call	 hca_cmd
	addl	$24 , %esp
	cmpl	$0xffffffff, %eax
	je		99f
	/* call mailbox read */
	/* place holder for the lower part of the 64 bit value returned from hca_cmd. */
	pushl 	$0x0
	/* opcode */
	pushl	$MLX4_CMD_MB_READ
	/* opcode modifier */
	pushl	$0 /*drek - URI to add */
	/* read MB offset from stack */
	movl	%ss:10(%ebp), %edx
	/* input_mod offset inside MB */
	pushl 	%edx
	/* in_param_l */
	pushl 	$0 
	/* in_param_h */
	pushl 	$0	
	/* invoke command */
	call	 hca_cmd
	addl	$20 , %esp
	/* pop lower part of the 64 bit that was read form MB */
	popl	%edx
	/* put the resault at the stack at in parameter as resault */
	movl 	%edx, %ss:14(%ebp)
99:
	/* pop previous edx/ebp value saved on stack */
	popl 	%edx
	popl 	%ebp
	ret

.size read_ini, . -read_ini

/*
 * Description - Read numer of ports using mod_stat
 * IN 	       - eax : port number
 * OUT 	       - eax : number of ports
 */
get_num_ports:
	/* place holder for the lower part of the 64 bit value returned from hca_cmd. */
	pushl $0x0
	/* opcode */
	pushl	$MLX4_CMD_MOD_STAT_CFG
	/* opcode modifier */ 
	pushl	$MOD_STAT_CFG_GET_INLINE_MOD
	/* input_mod  */
	shll 	$8, %eax
	orl  	$MOD_STAT_CFG_MAC_NUM_PORTS_OFFSET, %eax
	pushl 	%eax
	/* in_param_l */
	pushl 	$0 
	/* in_param_h */
	pushl 	$0	
	/* invoke command */
	call hca_cmd
	addl	$24, %esp
	cmpl    $0xffffffff, %eax
	je	99f
	shrl	$16, %eax
	andl	$0xff, %eax
99:
	ret
.size get_num_ports, . -get_num_ports 

/*
* IN -  eax : port number or 0 for globals
		%esp :
*			 [esp] + 2  = TLV_TYPE
*			 [esp] + 6  = TLV_ENTRY_OFFST
* OUT - eax : high of the 64 bit or FFFFFFFF on failure
*			[esp] + 10	= lower part of the 64 bit returned value from tlv
*/
get_tlv_value:
	/* save ebp register on stack */
	pushl 	%ebp
	/* update ebp value */
	movl 	%esp,%ebp
	/* save edx register on stack */
	pushl	%edx;
	/* place holder for the lower part of the 64 bit value returned from hca_cmd. */
	pushl	$0x0
	/* opcode */
	pushl	$MLX4_CMD_MOD_STAT_CFG
	/* opcode modifier */ 
	pushl	$MOD_STAT_CFG_GET_INLINE_FLASH_ACCESS_MOD
	/* input_mod (type modifier) */
	shll 	$16, %eax
	orl  	%ss:6(%ebp), %eax
	pushl 	%eax
	/* in_param_l */
	movl  	%ss:10(%ebp), %eax
	pushl 	%eax   /* dword offset */
	/* in_param_h */
	pushl 	$0	
	/* invoke command */
	call 	hca_cmd
	addl    $20 , %esp
	/* read returned lower part of the 64 bit value */
	popl	%edx
	/* return value from hca command forworded to caller */
	movl	%edx, %ss:14(%ebp)
	/* pop previous edx value saved on stack */
	popl	%edx
	/* pop previous ebp value saved on stack */
	popl	%ebp
	ret

.size get_tlv_value, . -get_tlv_value

/* hca_cmd
 *
 * In Parameters:
 *   %bx  : PCI bus:dev.fn -
 *   %di  : PCI register number - GW GW_ADDR_ADDR
 *   %esp : (esp + 2)  = HCR_INPUT_H_OFFSET
 *	    	(esp + 6)  = HCR_INPUT_L_OFFSET
 *	    	(esp + 10) =  HCR_INPUT_MOD_OFFSET
 *	    	(esp + 14) = HCR_OPCODE_MOD
 *	    	(esp + 18) = HCR_OPCODE
 *			(esp + 22) = lower part of the 64 bit returned value

 * Returns:
 *   %esp : (esp + 22) = output 64 bit low  Dword(inline)
 *   %eax : output 64 bit high Dword(inline)
 */
hca_cmd:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now paramters starts from (ebp + 6)) */
	movl 	%esp,%ebp

	pushl	%edi
	pushl	%edx
	pushl	%ecx
 	/* check sw ownership */
 	movw	$GW_ADDR_ADDR, %di
 	/* store shifted port in stack */
	call 	cmd_pend /*drek test case of failure */
	/* post command	*/
	/* write inline input h & l	*/
	movl    %ss:6(%ebp) , %ecx
	movl	$HCR_INPUT_H_OFFSET, %eax
	call	gw_write_dword

	movl    %ss:10(%ebp) , %ecx
	movl	$HCR_INPUT_L_OFFSET, %eax
	call	gw_write_dword
	/* write inline input modifier */
	movl    %ss:14(%ebp) , %ecx
	movl	$HCR_INPUT_MOD_OFFSET, %eax
	call	gw_write_dword
	/* write token	*/
	movl	$HCR_TOKEN , %ecx
	movl	$HCR_TOKEN_MOD_OFFSET, %eax
	call	gw_write_dword
	/* write hcr - post command	*/
	movl    %ss:18(%ebp) , %eax
	shll 	$HCR_OPMOD_SHIFT, %eax
	movl    %ss:22(%ebp) , %ecx
	orl	%eax, %ecx	
	orl	$HCR_GO_BIT_SET, %ecx
    movl	$HCR_OFFSET, %eax
	call	gw_write_dword
	/* pend & read result (to %eax)	*/
	call 	cmd_pend
	/* check status at eax!*/
	testl	%eax, %eax /* cmd status != 0 */
	jnz 	1f
	movl	$HCR_OUTPUT_L_OFFSET ,%eax
	call 	gw_read_dword
	movl 	%eax, %ss:26(%ebp)
	movl	$HCR_OUTPUT_H_OFFSET ,%eax
	call 	gw_read_dword
	jmp	2f
1:
	movl	$0xffffffff, %eax /* fail */
	movl	$0xffffffff, %ss:26(%ebp)/* fail */
2:
	popl	%ecx
	popl	%edx
	popl	%edi
	popl	%ebp
	ret
 	.size	hca_cmd, . - hca_cmd

/* Get PCI BAR value
 *
 * Parameters:
 *   %bx : PCI bus:dev.fn
 *   %di : PCI BAR register number
 * Returns:
 *   %edx:%eax : PCI BAR value
 */
my_pci_read_bar:
	/* Preserve registers */
	pushl	%ecx
	pushw	%di

	/* Read low dword value */
	call	my_pci_read_config_dword
	movl	%ecx, %eax

	/* Read high dword value, if applicable */
	xorl	%edx, %edx
	andb	$0x07, %cl
	cmpb	$0x04, %cl
	jne	1f
	addw	$4, %di
	call	my_pci_read_config_dword
	movl	%ecx, %edx
1:
	/* Restore registers and return */
	popw	%di
	popl	%ecx
	ret
	.size	my_pci_read_bar, . - my_pci_read_bar



/* gw_read_dword
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn -
 *   %di  : PCI register number - GW GW_ADDR_ADDR
 *	 %eax : HCR offset
 * Returns:
 *   %eax : result
 */
gw_read_dword: //offset at eax (HCR)!
	pushl   %ecx
	movl 	%eax, %ecx
	call	my_pci_write_config_dword
 	addw	$4, %di //data is there
 	call	my_pci_read_bar
 	subw	$4, %di

	popl	%ecx
 	ret
 	.size	gw_read_dword, . - gw_read_dword

/* gw_write_dword
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn -
 *   %di  : PCI register number - GW GW_ADDR_ADDR
 *	 %eax : IN - HCR offset , OUT - result
 *	 %ecx : IN - value to write
 * Returns:
 *   none
 */
gw_write_dword: //offset at eax (HCR)!
	pushl   %ecx
	movl 	%eax, %ecx
	call	my_pci_write_config_dword
	popl	%ecx

 	addw	$4, %di
 	call	my_pci_write_config_dword
 	subw	$4, %di

 	ret
 	.size	gw_write_dword, . - gw_write_dword

#else
#define TLV_ROM_BNR_TYPE 0x210

/*
 * Description - check device NVMEM configuration enabled
 * IN  	       -  NON
 * OUT 	       -  %eax:  1 if non-volatile config supported
 * side effects - the global parameter nv_cfg_en_f is updated according to eax
 */
check_nv_cfg_en:
	/* save edx register on stack */
	pushl	%edx
	movb	$0x1, nv_cfg_en_f
3:
	/* pop previous edx value saved on stack */
	popl	%edx
	ret
.size check_nv_cfg_en, . - check_nv_cfg_en


/*
 * Description - set flexboot menu time out from setting in TLV, if not exist
 				 then using ini, otherwise will use default.
 * IN  	       -  NON
 * OUT		   -  NON
 * side effects - flexboot_menu_to global parameter is set
 */
get_flexboot_menu_to:
	/* save edx/eax register to stack */
	pushl	%eax
	pushl	%edx
	/* low dword of 64 bit value */
	pushl	$0xffffffff
	/* push TLV entry offset */
	pushl	$TLV_ROM_BNR_ENTRY_OFST
	/* push TLV type to the stack */
	pushl	$TLV_ROM_BNR_TYPE
	/* port zero cause per host class */
	movl	$0x0, %eax
	call 	get_tlv_value
	/* pop passed parameter */
	addl    $8 , %esp
	/* put returned status in edx */
	movl	%eax, %edx
	/* wanted result should be in lower dword */
	popl	%eax
	/* check if return faild status */
	cmpl	$0xffffffff, %edx
	jne		1f
	movl	$DEFAULT_BANNER_TIMEOUT, %eax
1:
	/* %eax holds the wanted parameter */
	andl	$ROM_BANNER_TO_TLV_MASK, %eax
	movl	$0, %edx
	/* 18 * ax = (ax*16+ax*2) */
	movw	%ax, %dx
	shlw	$4, %ax
	shlw	$1, %dx
	addw	%dx, %ax
	movw	%ax,(flexboot_menu_to)
	/* pop registers previous values */
	popl	%edx
	popl	%eax
	ret
.size get_flexboot_menu_to, . - get_flexboot_menu_to

/*
* read TLV , no TLV read INI.
* if NO TLV & INI return dis (0)
*
* IN -  NONE
* OUT - eax :  en(1) / dis (0)
*
*/
#define BOOT_SETTINGS_TYPE 0x221
#define	OP_ROM_EN_INI_MASK 0x20000000
#define	OP_ROM_EN_TLV_MASK 0x80ff0000
#define	OP_ROM_EN_TLV_OFST 0x2
check_host_op_rom_en:
	/* save edx/eax/ecx register to stack */
	pushl	%ecx
	/* place holder low dword of 64 bit value, set default value to 1 */
	pushl	$0x1
	/* push TLV entry offset */
	pushl	$OP_ROM_EN_TLV_OFST
	/* push TLV type to the stack */
	pushl	$BOOT_SETTINGS_TYPE
	/* port zero cause per host class */
	movl	$0x0, %eax
	call 	get_tlv_value
	/* pop passed parameter */
	addl    $8, %esp
	/* pop resault */
	popl	%ecx
	/* check failure */
	cmpl	$0xffffffff, %eax
	je		1f
	/* align wanted word to first word */
	shrl	$16, %ecx
	/* legacy boot protocol check */
	andl	$0xff, %ecx
	jz		1f
	/* Value of 2 is the same as 1 - enabled */
   	movl	$1, %ecx
1:
	movl 	%ecx, %eax
99:
	popl	%ecx
	ret
.size check_host_op_rom_en, . -check_host_op_rom_en

#define MLX_ICMD_SEMAPHORE_ADDR 0x0
#define COMMON_SEMAPHORES_FLASH_PROGRAMMING_OFST 0x10
#define CAPABILITY_REG_OFFSET 0x34
#define CAPABILITY_OFFSET_MASK 0xff
#define MLX_CAPABILITY_ID 0x9
/* need to check if this offset is always valid */
#define MLX_SPECIFIC_CAP_OFST 0xc0
#define MLX_CAP_SEMAPHORE_OFST 0xc
#define MLX_CAP_COUNTER_OFST 0x8
#define MAX_NUM_TRIES 1024
#define MLX_CAP_SPACE_OFST 0x4
#define MLX_CAP_DATA_OFST 0x14
#define MLX_CAP_ADDR_OFST 0x10
#define MLX_CAP_ADDR_FLAG_MASK 0x80000000
//arbitrary PID ("FLXB")
#define	ROMPREFIX_PID	0x464c5842

#define MLX_ICMD_CTRL_ADDR 0x0
#define MLX_ICMD_MB_SIZE_ADDR 0x1000
#define MLX_ICMD_MB_ADDR 0x100000
#define FLASH_REG_ACCESS 0x9001
#define PCI_GW_SPACE_ALL_ICMD 0x3

#define ICMD_GW_OP_ALIGN 16
#define ICMD_OP_CODE_MASK 0x0000ffff
#define ICMD_BUSY_BIT_MASK	0x1
#define ICMD_GW_STATUS_MASK 0x0000ff00
#define ICMD_GW_STATUS_ALIGN 0x8
#define OPT_TLV_STAT_ALIGN 0x8
#define OPT_TLV_STAT_MASK 0x7F00

/* Len = 4 (fixed), type = 1 (fixed)*/
#define	OP_TLV_FIRST_DWORD 0x08040000
#define OP_TLV_REG_ID_ALIGN 16
#define OP_TLV_ACCESS_TYPE_ALIGN 8

#define REG_ACCESS_TLV_REG 3
#define REG_TLV_REG_ACCESS_ALIGN 27
#define REG_TLV_LENGTH_ALIGN 16
#define REG_TLV_BASE_OFFSET	MLX_ICMD_MB_ADDR+16

#define NV_ACCESS_BASE_OFFSET	MLX_ICMD_MB_ADDR+20
#define TLV_ENTRIES_BASE_OFFSET NV_ACCESS_BASE_OFFSET+12
#define NV_ACCESS_PORT_ALIGN 16

#define FLASH_REG_ACCESS 0x9001
#define OP_TLV_READ_ACCESS_TYPE 0x1
#define REG_ID_NVDA 0x9024
#define REG_ACCESS_TLV_REG 3

#define COMMON_SEMAPHORES_SPACE_ID 0x0A
#define OP_TLV_BASE_OFFSET MLX_ICMD_MB_ADDR

/* gw_get_cap_ofst
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 * Returns:
 *   %ax :  Mellanox Vendor Specific Capabilities entry offset,
 *	 on error: %ax = 0
 */
gw_get_cap_ofst:
	/* preserve registers values */
	pushl 	%ecx
	pushl 	%edi
	/* PCI register number (Capabilities ptr) */
	movw	$CAPABILITY_REG_OFFSET, %di
	/* Read dword value */
	call	my_pci_read_config_dword
	/* first capability entry offset read */
	andw	$CAPABILITY_OFFSET_MASK, %cx
	movw	%cx, %di
cap_loop:
	/* next pointer = zero then at the last capability, MLX id not found */
	testw	%di, %di
	jz		1f
	call	my_pci_read_config_dword
	cmpb	$MLX_CAPABILITY_ID, %cl
	je		1f

	/* shift right to align next pointer value*/
	shrw	$0x8, %cx
	movw	%cx, %di
	jmp		cap_loop

1:
	movw	%di, %ax
	movb	%al, mlx_cap_ofst

	/* Restore registers  values */
	popl	%edi
	popl	%ecx
	lret
.size	gw_get_cap_ofst, . - gw_get_cap_ofst

/* drek -  very dangerous loop */
/* gw_hold_ownership
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 * Returns:
 *   %eax: zero value if did not succeed to hold ownership, otherwise the
 *		   semaphore value
 */

gw_hold_ownership:
	/* Preserve registers values */
	pushl 	%ecx
	pushw	%di
	/* GW semaphore offset */
1:
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_SEMAPHORE_OFST, %di
	movl	$MAX_NUM_TRIES,%eax
check_gw_semaphore_val:
	/* about 0.25 sec delay */
	call	wait_for_tick
	decl	%eax
 	js		98f // Exit with ZF clear
	/* read semaphore value */
	call	my_pci_read_config_dword
	/* check semaphore value */
	testl	%ecx, %ecx
	jnz		check_gw_semaphore_val
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_COUNTER_OFST, %di
	/* read counter value */
	call	my_pci_read_config_dword
	/* update semaphore value with counter value */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_SEMAPHORE_OFST, %di
	call	my_pci_write_config_dword
	/*	confirm semaphore update value */
	/* read semaphore value */
	movl	%ecx, %eax
	call	my_pci_read_config_dword
	cmpl	%ecx, %eax
	jne		1b
	jmp 	99f
98:
	movl	$0x0, %eax
99:
	/* Restore registers  values */
	popw	%di
	popl	%ecx
	lret
.size	gw_hold_ownership, . - gw_hold_ownership

/* gw_release_ownership
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 * Returns:
 *   NONE
 */
gw_release_ownership:
	/* Preserve registers values */
	pushl 	%ecx
	pushw	%di
	/* GW semaphore offset */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_SEMAPHORE_OFST, %di
	/* write zero value to semaphore entry to release ownership */
	xorl	%ecx, %ecx
	call	my_pci_write_config_dword
	/* Restore registers  values */
	popw	%di
	popl	%ecx
	lret
.size	gw_release_ownership, . - gw_release_ownership

/* gw_write_to_space
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 *	 %ax  : space domain to access
 *	 %edx : offset in the selected address space (24 bit offset)
 *	 %esi : dword value to write
 * Returns:
 *   NONE
 * Notes:
 * 	 caller must hold GW ownership before calling this function.
 */
gw_write_to_space:
	/* Preserve registers values */
	pushl	%edi
	pushl	%ecx
	/* GW space offset */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_SPACE_OFST, %di
	xorl	%ecx, %ecx
	/* write space domain with status and reserved word 0 */
	movw	%ax, %cx
	call	my_pci_write_config_dword

	/* write data dword */
	/* GW data offset */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_DATA_OFST, %di
	/* data to write */
	movl	%esi, %ecx
	/* write data */
	call	my_pci_write_config_dword

	/* set flag bit to monitor later for successful space write access */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_ADDR_OFST, %di
	movl	$MLX_CAP_ADDR_FLAG_MASK, %ecx
	/* offset in the selected address space mov to ecx with flag bit set */
	orl		%edx, %ecx
	call	my_pci_write_config_dword

	/* monitor flag bit value till zero to indicate success */
flag_clr:
	call	my_pci_read_config_dword
	andl	$MLX_CAP_ADDR_FLAG_MASK, %ecx
	jnz		flag_clr
	/* Restore registers  values */
	popl	%ecx
	popl	%edi
	ret
.size	gw_write_to_space, . - gw_write_to_space

/* gw_read_from_space
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 *	 %ax  : space domain to access
 *	 %edx : offset in the selected address space (24 bit offset)
 * Returns:
 *	 %eax : dword value that was read from space
 * Notes:
 * caller must hold GW ownership before calling this function.
 */
gw_read_from_space:
	/* Preserve registers values */
	pushl	%edi
	pushl	%ecx
	/* GW space offset */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_SPACE_OFST, %di
	xorl	%ecx, %ecx
	/* write space domain with status and reserved word 0 */
	movw	%ax, %cx
	call	my_pci_write_config_dword
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_ADDR_OFST, %di
	xorl	%ecx, %ecx
	/* offset in the selected address space mov to ecx with flag bit cleared */
	orl		%edx, %ecx
	call	my_pci_write_config_dword
	/* monitor flag bit value till nonzero value to indicate success */
flag_set:
	call	my_pci_read_config_dword
	andl	$MLX_CAP_ADDR_FLAG_MASK, %ecx
	jz		flag_set
	/* read data returned from space to GW data entry */
	/* GW data offset */
	movzbw	mlx_cap_ofst, %di
	addw	$MLX_CAP_DATA_OFST, %di
	call	my_pci_read_config_dword
	/* return data in %eax */
	movl	%ecx, %eax
	/* Restore registers  values */
	popl	%ecx
	popl	%edi
	lret
.size	gw_read_from_space, . - gw_read_from_space

/* hold_common_semaphore
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 *	 %edx : semaphore offset in common semaphore space
 * Returns:
 *	 %eax :
 *			non zero proccess id - on success
 *			0 - on failure
 * Notes:
 * caller must hold GW ownership before calling
 * this function.
 */
hold_common_semaphore:
	/* Preserve registers values */
	pushl	%ecx
	pushl	%esi
	/* set default PID in case of failure */
	movl	$ROMPREFIX_PID,%esi
	pushl	%edx
	xorl	%eax,%eax
	/* Get random PID by sampling BIOS RTC in cycles */
	movb	$0x0,%ah
	int	$0x1a
	movzwl	%cx,%eax
	shll	$16,%eax
	movw	%dx,%ax
	movl	%eax,%esi
	testl	%esi,%esi
	jnz		1f
	movl	$ROMPREFIX_PID,%esi
1:
	popl	%edx
2:
	movl	$MAX_NUM_TRIES,%ecx
check_common_semaphore_val:
	/* about 0.25 sec delay */
	call	wait_for_tick
	decl	%ecx
	js		3f
	movw	$COMMON_SEMAPHORES_SPACE_ID,%ax
	pushw	%cs
	call	gw_read_from_space
	/* if semaphore value is not zero retry to hold */
	testl	%eax, %eax
	jnz		check_common_semaphore_val
	movw	$COMMON_SEMAPHORES_SPACE_ID,%ax
	/* PID value is already in %esi */
	call	gw_write_to_space
	/* verify that semaphore in common space semaphore is locked */
	pushw	%cs
	call	gw_read_from_space
	cmpl	%esi,%eax
	jne		2b
	jmp 	4f
3:
	movl	$0x0, %eax
4:
	/* Restore registers  values */
	popl	%esi
	popl	%ecx
	ret
.size	hold_common_semaphore, . - hold_common_semaphore

/* release_common_semaphore
 *
 * Parameters:
 *   %bx  : PCI bus:dev.fn
 *	 %edx : semaphore offset in common semaphore space
 * Returns:
 * NONE
 * Notes:
 * caller must hold GW ownership before calling
 * this function.
 */
 release_common_semaphore:
	/* Preserve registers values */
	pushl	%esi
	pushw	%ax
 	movw	$COMMON_SEMAPHORES_SPACE_ID,%ax
 	/*write 0 at semaphore offset in common space to release semaphore */
	movl	$0x0, %esi
	call	gw_write_to_space
	/* Restore registers  values */
	popw	%ax
	popl	%esi
	ret
.size	release_common_semaphore, . - release_common_semaphore


/*
*	icmd_access
*		%esp :
*			 [esp] + 2  = ICMD opcode
*			 [esp] + 6  = busy bit (busy = 1 ==> will monitor till 0)
* OUT - eax :
*			eax = status from OPT_TLV if busy bit = 1 at input, if already busy
*				  return 0xffffffff
* Notes:
* caller must hold GW ownership and appropriate common semaphore before calling
* this function.
*/
icmd_access:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now parameters starts from (ebp + 6)) */
	movl 	%esp,%ebp
	/* Preserve registers values */
	pushl	%edx
	pushl	%esi
	/* read existing ICMD from space */
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$MLX_ICMD_CTRL_ADDR, %edx
	pushw	%cs
	call	gw_read_from_space
	/* check if busy then return error */
	testl	$0x1, %eax
	jne		1f	//if result not zero then jump
	/* not busy so mask opcode and write new one */
	andl	$ICMD_OP_CODE_MASK, %eax
	movl	%ss:6(%ebp), %esi
	shll	$ICMD_GW_OP_ALIGN, %esi
	orl		%eax, %esi
	/* if busy bit need to be set then set busy bit */
	orl		%ss:10(%ebp), %esi
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$MLX_ICMD_CTRL_ADDR, %edx
	call	gw_write_to_space
	/* if busy bit at input is 1 then monitor and return status */
	testl		$ICMD_BUSY_BIT_MASK, %esi
	/* if busy bit not set then just return */
	je		2f /*if busy bit is zero then just return without monitor so the
				 prints won't come up */
	/* busy bit is set so monitor it till bit is zero */
icmd_busy_bit_mon:
	/* about 0.25 sec delay */
	call	wait_for_tick
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	/* space already set */
	pushw	%cs
	call	gw_read_from_space
	testl	$ICMD_BUSY_BIT_MASK, %eax
	jnz 	icmd_busy_bit_mon
	/* read status if not zero (SUCCESS code) abort */
	testl	$ICMD_GW_STATUS_MASK, %eax
	/* failure  and eax hold the status on jmp */
	jne 	2f

	/*return status at op_tlv (need to be 0 for success otherwise failure) */
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$OP_TLV_BASE_OFFSET, %edx
	pushw	%cs
	call 	gw_read_from_space
	/* mask and align status to return to caller */
	andl	$OPT_TLV_STAT_MASK, %eax
	shrl	$OPT_TLV_STAT_ALIGN, %eax

	jmp		3f

1:
	/* if already running ICMD */
	movl	$0xffffffff,%eax
	jmp		3f

2:

	shrl	$ICMD_GW_STATUS_ALIGN, %eax

3:
	/* restore registers values */
	popl	%esi
	popl	%edx
	popl	%ebp
	ret
.size icmd_access, . - icmd_access

/*
*	write_op_tlv
*		%esp :
*			  [esp] + 2  = REG_ID
*			  [esp] + 6  = access_type (read = 1, write = 2)
* OUT 		 :
*			  NONE
* Notes:
* caller must hold GW ownership and appropriate common semaphore before calling
* this function.
*/
write_op_tlv:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now parameters starts from (ebp + 6)) */
	movl 	%esp,%ebp
	/* Preserve registers values */
	pushl	%edx
	pushl	%esi
	pushl	%eax

	/* write first dword */
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$OP_TLV_BASE_OFFSET, %edx
	movl	$OP_TLV_FIRST_DWORD, %esi
	call	gw_write_to_space

	/* write second dword */
	addl	$0x4, %edx
	movl	%ss:6(%ebp), %eax
	shll	$OP_TLV_REG_ID_ALIGN, %eax
	movl	%ss:10(%ebp), %esi
	shll	$OP_TLV_ACCESS_TYPE_ALIGN, %esi
	/* set cls = 1 */
	orl		$0x1, %esi
	orl		%eax, %esi
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	call	gw_write_to_space

	/* write third dword value 0 */
	xorl	%esi,%esi
	addl	$0x4, %edx
	call	gw_write_to_space
	/* write forth dword value 0 */
	addl	$0x4, %edx
	call	gw_write_to_space
	/* restore registers values */
	popl	%eax
	popl	%esi
	popl	%edx
	popl	%ebp
	ret
.size write_op_tlv, . - write_op_tlv

/*
*	write_reg_access
*		%esp :
*			  [esp] + 2  = reg_access_type
*			  [esp] + 6  = Len field in Reg_tlv_word
* OUT 		 :
*			  NONE
* Notes:
* caller must hold GW ownership and appropriate common semaphore before calling
* this function.
*/
write_reg_access:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now parameters starts from (ebp + 6)) */
	movl 	%esp,%ebp
	/* Preserve registers values */
	pushl	%edx
	pushl	%esi
	pushl	%eax

	/* reg access type save to eax */
	movl	%ss:6(%ebp), %eax
	shll	$REG_TLV_REG_ACCESS_ALIGN, %eax
	/* read len sizeof(NV_ACCESS_HEADER + NV_ACCESS_DATA + REG_TLV_HEADER)/4 */
	movl	%ss:10(%ebp), %esi
	shll	$REG_TLV_LENGTH_ALIGN, %esi
	orl		%eax, %esi
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$REG_TLV_BASE_OFFSET, %edx
	call	gw_write_to_space
	/* restore registers values */
	popl	%eax
	popl	%esi
	popl	%edx
	popl	%ebp

	ret
.size write_reg_access, . - write_reg_access


/*
*	write_nv_access
*		%esp :
*			  [esp] + 2  = TLV_TYPE
*			  [esp] + 6  = PORT_NUM
*			  [esp] + 10  = length of data afterwords in bytes
* OUT 		 :
*			  NONE
* Notes:
* caller must hold GW ownership and appropriate common semaphore before calling
* this function.
*/
write_nv_access:
	/* save ebp register to stack */
	pushl 	%ebp
	/* update ebp value (now parameters starts from (ebp + 6)) */
	movl 	%esp,%ebp
	/* Preserve registers values */
	pushl	%edx
	pushl	%esi
	pushl	%eax

	/* first dword all zero except over_en bit */
	movl	$0x02000000, %esi
	orl		%ss:14(%ebp), %esi
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$NV_ACCESS_BASE_OFFSET, %edx
	call	gw_write_to_space

	/* write second dword */
	/* save port number and then align */
	movl	%ss:10(%ebp), %esi
	shll	$NV_ACCESS_PORT_ALIGN, %esi
	/* TLV type in first byte */
	orl		%ss:6(%ebp), %esi
	/* set param class byte ( set to per host) */
	orl		$0x03000000, %esi
	addl	$0x4, %edx
	call	gw_write_to_space

	/* write third dword value 0 */
	xorl	%esi,%esi
	addl	$0x4, %edx
	call	gw_write_to_space
	/* restore registers values */
	popl	%eax
	popl	%esi
	popl	%edx
	popl	%ebp

	ret
.size write_nv_access, . - write_nv_access

#define FW_INIALIZATION_TIMEOUT_IN_TICKS 180
#define PCI_GW_SPACE_NODNIC 0x4
#define PCI_GW_SPACE_NODNIC_INIT_DWORD_OFST 0x1fc
#define INIT_BIT_MASK	0x80000000
/* wait_fw_up:
*	waits until firmware up bit at Nodnic space (offset 0x1fc) is cleared
*	or a timeout of 10 sec is up.
*
* Parameters:
*   %bx  : PCI bus:dev.fn
* Returns:
*   %eax: 1 = if timeout expired and fw initialization bit is not cleared.
*		  0 = fw initialization bit is cleared.
*/
wait_fw_up:
	pushl	%edx
	pushl	%esi
	/* hold gw ownership */
	pushw	%cs
	call	gw_hold_ownership
	testl	%eax, %eax
	jz 		99f
	movl	$FW_INIALIZATION_TIMEOUT_IN_TICKS, %esi
retry_check:
	/* Nodnic space */
	movw	$PCI_GW_SPACE_NODNIC, %ax
	/* offset in Nodnic space */
	movl	$PCI_GW_SPACE_NODNIC_INIT_DWORD_OFST,%edx
	pushw	%cs
	call	gw_read_from_space
	andl	$INIT_BIT_MASK, %eax
	testl	%eax, %eax
	jz		1f
	call	wait_for_tick
	decl	%esi
	jns		retry_check /* jump if ZF not cleared */
	movl	$0x1, %eax
1:
	pushw	%cs
	call	gw_release_ownership
99:
	popl	%esi
	popl	%edx
	ret

.size wait_fw_up, . - wait_fw_up

/*
* IN -  eax : port number or 0 for NON per port settings
*		bx  : PCI bus:dev.fn
*		esp :
*			 [esp] + 2  = TLV_TYPE
*			 [esp] + 6  = TLV_ENTRY_OFFST
* OUT - eax : high of the 64 bit or FFFFFFFF on failure
*			[esp] + 10	= lower part of the 64 bit returned value from tlv
*						 or zero on case of error.
* Notes:
* if GW hold ownership fails, then [esp] + 10 will not set to zero,
* to keep option of default value in case of hold ownership failure.
*/
get_tlv_value:
/* save ebp register on stack */
	pushl 	%ebp
	/* update ebp value (parameters starts at ebp + 6) */
	movl 	%esp,%ebp
	/* Preserve registers values */
	pushl	%edx
	pushl	%esi
	/* save port number in esi cause gw_hold_ownership may change %eax value*/
	movl	%eax, %esi
	/* hold gw ownership */
	pushw	%cs
	call	gw_hold_ownership
	movl	%eax, %edx
	movl	$0xffffffff, %eax
	testl	%edx, %edx
	/* exit if failed to hold owner ship */
	jz 		99f
	/* return value of 0 in case of any error from this point on */
	movl	$0x0, %ss:14(%ebp)
	/* hold "Tools ICMD" semaphore in common semaphore space */
	movl	$MLX_ICMD_SEMAPHORE_ADDR,%edx
	call 	hold_common_semaphore
	movl	%eax, %edx
	movl	$0xffffffff, %eax
	testl	%edx, %edx
	/* exit if failed to hold ICMD semaphore */
	jz 		3f
	/* by default return 1 on stack */
	movl	$0x1, %ss:14(%ebp)
	/* busy is set to 0 */
	pushl	$0x0
	/* set op code */
	pushl	$FLASH_REG_ACCESS
	call 	icmd_access
	addl	$0x8, %esp

	/* access_type read ( = 1) */
	pushl	$OP_TLV_READ_ACCESS_TYPE
	/* set op code */
	pushl	$REG_ID_NVDA
	call 	write_op_tlv
	addl	$0x8, %esp

	/* REG_ACCESS  */
	/*len = 5  = REg_ACCESS_HDR (1)+ NV_ACCESS_header(3) + 1 dword data*/
	pushl	$0x5
	pushl	$REG_ACCESS_TLV_REG
	call 	write_reg_access
	addl	$0x8, %esp

	/* write NV_ACCESS field */
	/* data size is one dword (4  bytes)*/
	pushl	$0x04
	/* port number 1 (should be taken from %eax at input) */
	pushl	%esi
	/* TLV_TYPE parameter form stack */
	movl	%ss:6(%ebp), %esi
	pushl	%esi
	call 	write_nv_access
	addl	$12, %esp
	/* run icmd give a go bit %eax will hold the return code*/
	/* busy is set to 1 */
	pushl	$0x1
	/* set op code */
	pushl	$FLASH_REG_ACCESS
	call 	icmd_access
	addl	$0x8, %esp
	cmpl	$0x0, %eax
	jne		1f
	/* read MB (lower dword) will be on stack */
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	movl	$TLV_ENTRIES_BASE_OFFSET, %edx
	addl	%ss:10(%ebp), %edx
	pushw	%cs
	call	gw_read_from_space
	movl	%eax, %ss:14(%ebp)
	/* read MB (high dword) will be returned in %eax */
	movw	$PCI_GW_SPACE_ALL_ICMD, %ax
	addl	$0x4, %edx
	pushw	%cs
	call	gw_read_from_space
	jmp		2f
1:
	/* error code */
	movl	$0xffffffff, %eax

2:
	/* release "Tools ICMD" semaphore in common semaphore space */
	movl	$MLX_ICMD_SEMAPHORE_ADDR, %edx
	call 	release_common_semaphore
3:
	/* release gw ownership */
	pushw	%cs
	call 	gw_release_ownership
99:
	/* restore registers values */
	popl	%esi
	popl	%edx
	popl	%ebp
	ret
.size get_tlv_value, . -get_tlv_value

#endif //ifndef DEVICE_CX3
#endif

#ifdef MLX_BUILD
driver_version:
 .align 16
 .long 0x73786c6d
 .long 0x3a6e6769
 .long __MLX_0001_MAJOR_VER_
 .long __MLX_MIN_SUB_MIN_VER_
 .long __MLX_DEV_ID_00ff
#endif

/* Drag in objects via _rom_start */
REQUIRING_SYMBOL ( _rom_start )

/* Drag in ROM configuration */
REQUIRE_OBJECT ( config_romprefix )
