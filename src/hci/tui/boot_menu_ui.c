/*
 * Copyright (C) 2014-2015 Mellanox Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <curses.h>
#include <ipxe/console.h>
#include <ipxe/settings.h>
#include <ipxe/editbox.h>
#include <ipxe/keys.h>
#include <ipxe/ansicol.h>
#include <ipxe/boot_menu_ui.h>
#include <ipxe/driver_settings.h>
#include <ipxe/menu.h>
#include <ipxe/shell.h>

uint16_t __data16 ( boot_post_shell );

uint8_t boot_menu_rw = 0;

/* Screen layout */
#define TITLE_ROW		1U
#define SETTINGS_LIST_ROW	3U
#define SETTINGS_LIST_COL	1U
#define SETTINGS_LIST_ROWS	( LINES - 6U - SETTINGS_LIST_ROW )
#define INFO_ROW		( LINES - 5U )
#define ALERT_ROW		( LINES - 2U )
#define INSTRUCTION_ROW		( LINES - 2U )
#define INSTRUCTION_PAD "  "

/* Return options for esc menu */
#define BACK 1
#define EXIT 2
#define SHELL 3

/** Layout of text within a setting widget */
#define SETTING_ROW_TEXT( cols, len ) struct {				\
	char start[0];							\
	char pad1[1];							\
	union {								\
		char settings[ cols - 1 - 1 - 1 - 1 ];			\
		struct {						\
			char name[len];					\
			char pad2[1];					\
			char value[ cols -1 -len -1 -1 -1 -1 ];		\
		} setting;						\
	} u;								\
	char pad3[1];							\
	char nul;							\
} __attribute__ (( packed ))

/** A setting row widget */
struct setting_row_widget {
	/** Target configuration settings block
	 *
	 * Valid only for rows that lead to new settings blocks.
	 */
	struct settings *settings;
	/** Configuration setting origin
	 *
	 * Valid only for rows that represent individual settings.
	 */
	struct settings *origin;
	/** Configuration setting
	 *
	 * Valid only for rows that represent individual settings.
	 */
	struct setting setting;
	/** Screen row */
	unsigned int row;
	/** Screen column */
	unsigned int col;
	/** Edit box widget used for editing setting */
	struct edit_box editbox;
	/** Editing in progress flag */
	int editing;
	/** Buffer for setting's value */
	char value[256]; /* enough size for a DHCP string */
};

/** A settings widget */
struct setting_widget {
	/** Settings block */
	struct settings *settings;
	/** Number of rows */
	unsigned int num_rows;
	/** Current row index */
	unsigned int current;
        /** Index of the first visible row, for scrolling. */
	unsigned int first_visible;
	/** Active row */
	struct setting_row_widget row;
	/** Row's name length **/
	unsigned int row_len;
};

struct choose_options {
        /** Menu name */
        const char *menu;
        /** Timeout */
        unsigned int timeout;
        /** Default selection */
        const char *select;
        /** Keep menu */
        int keep;
};


static void clearmsg ( unsigned int row );
static void msg ( unsigned int row, const char *fmt, ...);
/**
 * Select a setting row
 *
 * @v widget		Setting widget
 * @v index		Index of setting row
 * @ret count		Number of settings rows
 */
static unsigned int select_setting_row ( struct setting_widget *widget,
					 unsigned int index ) {
	SETTING_ROW_TEXT ( COLS, widget->row_len ) *text;
	struct settings *settings;
	struct setting *setting;
	struct setting *previous = NULL;
	unsigned int count = 0;


	/* Initialise structure */
	memset ( &widget->row, 0, sizeof ( widget->row ) );
	widget->current = index;
	widget->row.row = ( SETTINGS_LIST_ROW + index - widget->first_visible );
	widget->row.col = SETTINGS_LIST_COL;

	/* Include any child settings blocks, if applicable */
	list_for_each_entry ( settings, &widget->settings->children, siblings ) {
		/* Skip hidden settings block */
		if ( settings->hidden )
			continue;
		if ( count++ == index ) {
			widget->row.settings = settings;
			snprintf ( widget->row.value,
				   sizeof ( widget->row.value ), "%s",
				   settings->name );
		}
	}

	/* Include any applicable settings */
	for_each_table_entry ( setting, SETTINGS ) {
		/* Skip hidden settings */
		if ( setting->hidden )
			continue;

		/* Skip inapplicable settings */
		if ( ! setting_applies ( widget->settings, setting ) )
			continue;

		/* Skip duplicate settings */
		if ( previous && ( setting_cmp ( setting, previous ) == 0 ) )
			continue;
		previous = setting;

		/* Read current setting value and origin */
		if ( count++ == index ) {
			fetchf_setting ( widget->settings, setting,
					 &widget->row.origin,
					 &widget->row.setting,
					 widget->row.value,
					 sizeof ( widget->row.value ) );
		}
	}

	/* Initialise edit box */
	init_editbox ( &widget->row.editbox, widget->row.value,
		       sizeof ( widget->row.value ), NULL, widget->row.row,
		       ( widget->row.col +
			 offsetof ( typeof ( *text ), u.setting.value ) ),
		       sizeof ( text->u.setting.value ), 0 );

	return count;
}

/**
 * Copy string without NUL termination
 *
 * @v dest		Destination
 * @v src		Source
 * @v len		Maximum length of destination
 * @ret len		Length of (unterminated) string
 */
static size_t string_copy ( char *dest, const char *src, size_t len ) {
	size_t src_len;

	src_len = strlen ( src );
	if ( len > src_len )
		len = src_len;
	memcpy ( dest, src, len );
	return len;
}

#define GET_EXTENDED_TYPE( ext, settings )				\
	do {								\
		if ( ext->get_extended_type )				\
			type = ext->get_extended_type ( settings );	\
		else							\
			type = ext->type;				\
	} while(0)							\

/**
 * Draw setting row
 *
 * @v widget		Setting widget
 */
static void draw_setting_row ( struct setting_widget *widget ) {
	SETTING_ROW_TEXT ( COLS, widget->row_len ) text;
	struct extended_setting *ext;
	unsigned int curs_offset;
	char *value;
	int type;

	/* Fill row with spaces */
	memset ( &text, ' ', sizeof ( text ) );
	text.nul = '\0';

	/* Construct row content */
	if ( widget->row.settings ) {

		/* Construct space-padded name */
		curs_offset = ( offsetof ( typeof ( text ), u.settings ) +
				string_copy ( text.u.settings,
					      widget->row.value,
					      sizeof ( text.u.settings ) ) );

	} else {
		/* Construct dot-padded name */
		memset ( text.u.setting.name, '.',
			 	sizeof ( text.u.setting.name ) );
		string_copy ( text.u.setting.name, widget->row.setting.description,
		      			sizeof ( text.u.setting.name ) );

		/* Construct space-padded value */
		value = widget->row.value;
		if ( ! *value )
			value = "<not specified>";
		curs_offset = ( offsetof ( typeof ( text ), u.setting.value ) +
				string_copy ( text.u.setting.value, value,
					      sizeof ( text.u.setting.value )));
	}

	/* Print row */
	if ( !widget->row.setting.name ) {
		attron ( A_BOLD );
	} else {
		ext = find_extended ( &widget->row.setting );
		GET_EXTENDED_TYPE( ext, widget->settings );
		if ( type != LABEL )
			attron ( A_BOLD );
	}

	mvprintw ( widget->row.row, widget->row.col, "%s", text.start );
	attroff ( A_BOLD );
	move ( widget->row.row, widget->row.col + curs_offset );
}

static int change_setting ( struct setting_widget *widget ) {
	assert ( widget->row.setting.name != NULL );
	struct extended_setting* ext;
	struct options_list* options;

	ext = find_extended ( &widget->row.setting);
	options = (struct options_list*)(ext->data);
	options->current_index++;
	if ( options->current_index >= options->options_num ) {
		options->current_index = 0;
	}
	strcpy (widget->row.value, options->options_array[options->current_index]);
	return storef_setting ( widget->settings, &widget->row.setting,
                                widget->row.value );
}


/**
 * Edit setting widget
 *
 * @v widget		Setting widget
 * @v key		Key pressed by user
 * @ret key		Key returned to application, or zero
 */
static int edit_setting ( struct setting_widget *widget, int key ) {
	assert ( widget->row.setting.name != NULL );
	widget->row.editing = 1;
	return edit_editbox ( &widget->row.editbox, key );
}

/**
 * Save setting widget value back to configuration settings
 *
 * @v widget		Setting widget
 */
static int save_setting ( struct setting_widget *widget ) {

	assert ( widget->row.setting.name != NULL );
	return storef_setting ( widget->settings, &widget->row.setting,
				widget->row.value );
}

/**
 * Print message centred on specified row
 *
 * @v row		Row
 * @v fmt		printf() format string
 * @v args		printf() argument list
 */
static void vmsg ( unsigned int row, const char *fmt, va_list args ) {
	char buf[COLS];
	size_t len;

	len = vsnprintf ( buf, sizeof ( buf ), fmt, args );
	mvprintw ( row, ( ( COLS - len ) / 2 ), "%s", buf );
}

/**
 * Print message centred on specified row
 *
 * @v row		Row
 * @v fmt		printf() format string
 * @v ..		printf() arguments
 */
static void msg ( unsigned int row, const char *fmt, ... ) {
	va_list args;

	va_start ( args, fmt );
	vmsg ( row, fmt, args );
	va_end ( args );
}

/**
 * Clear message on specified row
 *
 * @v row		Row
 */
static void clearmsg ( unsigned int row ) {
	move ( row, 0 );
	clrtoeol();
}

/**
 * Print alert message
 *
 * @v fmt		printf() format string
 * @v args		printf() argument list
 */
static void valert ( const char *fmt, va_list args ) {
	clearmsg ( ALERT_ROW );
	color_set ( CPAIR_ALERT, NULL );
	vmsg ( ALERT_ROW, fmt, args );
	sleep ( 2 );
	color_set ( CPAIR_NORMAL, NULL );
	clearmsg ( ALERT_ROW );
}

/**
 * Print alert message
 *
 * @v fmt		printf() format string
 * @v ...		printf() arguments
 */
static void alert ( const char *fmt, ... ) {
	va_list args;

	va_start ( args, fmt );
	valert ( fmt, args );
	va_end ( args );
}

/**
 * Draw title row
 *
 * @v widget		Setting widget
 */
static void draw_title_row ( struct setting_widget *widget ) {

	clearmsg ( TITLE_ROW );
	attron ( A_BOLD );
	msg ( TITLE_ROW, "%s", widget->settings->name );
	attroff ( A_BOLD );
}

/**
 * Draw information row
 *
 * @v widget		Setting widget
 */
static void draw_info_row ( struct setting_widget *widget ) {
	struct extended_setting* ext;
	int type;

	/* Draw nothing unless this row represents a setting */
	clearmsg ( INFO_ROW );
	clearmsg ( INFO_ROW + 1 );
	if ( ! widget->row.setting.name )
		return;

	/* Draw row */
	attron ( A_BOLD );
	msg ( INFO_ROW, "%s", widget->row.setting.description );
	attroff ( A_BOLD );

	ext = find_extended (&widget->row.setting);
	GET_EXTENDED_TYPE( ext, widget->settings );
	if ( ext->instructions ) {
		msg ( ( INFO_ROW + 1 ), "%s", ( char * )( ext->instructions ) );
	} else if ( type == OPTION ) {
		msg ( ( INFO_ROW + 1 ), "Select to change value" );
	} else {
		/* no instructions */
	}
}

/**
 * Draw instruction row
 *
 * @v widget		Setting widget
 */
#define STR_SAVE_AND_EXIT		"Ctrl-S - Save and exit"
#define STR_ACCEPT_CHANGES		"Enter - Accept changes"
#define STR_DISCARD_CHANGES		"Ctrl-C - Discard changes"
#define STR_RESTORE_PORT_DEFAULTS	"Ctrl-R - Restore port default configurations"
#define STR_RESTORE_DEVICE_DEFAULTS 	"Ctrl-R - Restore device default configurations"
#define STR_DELETE_SETTING		"Ctrl-D - Delete setting"
static void draw_instruction_row ( struct setting_widget *widget ) {
	struct extended_setting *ext;
	int type;

	clearmsg ( INSTRUCTION_ROW );

	if ( widget->row.editing ) {
		msg ( INSTRUCTION_ROW,
			STR_ACCEPT_CHANGES INSTRUCTION_PAD
			STR_DISCARD_CHANGES );
	} else if ( widget->row.settings && widget->settings->default_scope == &main_scope  ) {
		msg ( INSTRUCTION_ROW,
			STR_SAVE_AND_EXIT INSTRUCTION_PAD
			STR_RESTORE_DEVICE_DEFAULTS );
	} else if ( widget->row.settings && widget->settings->default_scope == &port_scope  ) {
                msg ( INSTRUCTION_ROW,
                        STR_SAVE_AND_EXIT INSTRUCTION_PAD
                        STR_RESTORE_PORT_DEFAULTS );
	} else if ( widget->row.settings ) {
		msg ( INSTRUCTION_ROW,
                        STR_SAVE_AND_EXIT );
	} else {
		ext = find_extended ( &widget->row.setting );
		GET_EXTENDED_TYPE( ext, widget->settings );
		if ( type == LABEL ) {
			msg ( INSTRUCTION_ROW,
			STR_SAVE_AND_EXIT INSTRUCTION_PAD );
		} else { /* Type is options or input */
			msg ( INSTRUCTION_ROW,
			STR_SAVE_AND_EXIT INSTRUCTION_PAD
			STR_DELETE_SETTING );
		}
	}
}

/**
 * Reveal setting row
 *
 * @v widget		Setting widget
 * @v index		Index of setting row
 */
static void reveal_setting_row ( struct setting_widget *widget,
				 unsigned int index ) {
	unsigned int i;

	/* Simply return if setting N is already on-screen. */
	if ( index - widget->first_visible < SETTINGS_LIST_ROWS )
		return;

	/* Jump scroll to make the specified setting row visible. */
	while ( widget->first_visible < index )
		widget->first_visible += SETTINGS_LIST_ROWS;
	while ( widget->first_visible > index )
		widget->first_visible -= SETTINGS_LIST_ROWS;

	/* Draw ellipses before and/or after the settings list to
	 * represent any invisible settings.
	 */
	mvaddstr ( SETTINGS_LIST_ROW - 1,
		   SETTINGS_LIST_COL + 1,
		   widget->first_visible > 0 ? "..." : "   " );
	mvaddstr ( SETTINGS_LIST_ROW + SETTINGS_LIST_ROWS,
		   SETTINGS_LIST_COL + 1,
		   ( ( widget->first_visible + SETTINGS_LIST_ROWS )
		     < widget->num_rows ? "..." : "   " ) );

	/* Draw visible settings. */
	for ( i = 0; i < SETTINGS_LIST_ROWS; i++ ) {
		if ( ( widget->first_visible + i ) < widget->num_rows ) {
			select_setting_row ( widget,
					     widget->first_visible + i );
			draw_setting_row ( widget );
		} else {
			clearmsg ( SETTINGS_LIST_ROW + i );
		}
	}
}

static void set_row_len ( struct setting_widget *widget ) {
	if ( widget->settings->default_scope == &iscsi_init_scope
	  || widget->settings->default_scope == &iscsi_target_scope ) {
		widget->row_len = 25;
	} else {
		widget->row_len = 35;
	}
}

/**
 * Reveal setting row
 *
 * @v widget		Setting widget
 * @v settings		Settings block
 */
static void init_widget ( struct setting_widget *widget,
			  struct settings *settings ) {
	widget->settings = settings_target ( settings );
	set_row_len ( widget );
	widget->num_rows = select_setting_row ( widget, 0 );
	widget->first_visible = SETTINGS_LIST_ROWS;
	draw_title_row ( widget );
	reveal_setting_row ( widget, 0 );
	select_setting_row ( widget, 0 );
}

static void store_settings_to_flash () {
	printf ("Saving settings ... ");

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return;
	}

	driver_settings_nv_store ();

	printf ("Done\n");
}

static void enable_driver_settings_store () {
        struct driver_settings *driver_settings;

	if ( ! nv_settings_root ) {
		printf ( "%s: nv_settings_root is not initialized\n", __FUNCTION__ );
		return;
	}

	list_for_each_entry ( driver_settings, &driver_settings_list, list )
		driver_settings->store_menu = 1;

}

static void restore_all_defaults () {
	struct driver_settings *driver_settings;
	list_for_each_entry ( driver_settings, &driver_settings_list, list ) {
		if ( driver_settings->callbacks.set_default )
			driver_settings->callbacks.set_default ( driver_settings );
	}
}

static void restore_defaults ( struct settings *settings ) {
	struct driver_settings *driver_settings = get_driver_settings_from_settings ( settings );
	if ( driver_settings->callbacks.set_default )
		driver_settings->callbacks.set_default ( driver_settings );
}

#define STR_SAVE		"save"
#define STR_DISCARD		"discard"
#define STR_BACK		"back"
#define STR_EXIT		"Exit"
#define STR_EXIT_TO_SHELL_TITLE	""
#define STR_EXIT_TO_SHELL	"Exit to shell"
#define STR_CONTINUE_BOOT	"Continue boot"
#define STR_SAVE_CONF		"Save configurations"
#define STR_ESC_DISCARD_CHANGES	"Discard changes"
#define STR_GO_BACK		"Go back to devices configuration menu"
#define STR_YES			"Yes"
#define STR_NO			"No"
#define SELECTED_STR_MAX_LEN	10
#define TOTAL_MENU_ITEMS	3
#define ESC_MENU_ADD_ITEM( label, desc )				\
	do {								\
		item = add_menu_item ( menu, label, desc, j++ , 0 );	\
		if ( ! item )						\
			return menu;					\
	} while ( 0 )

struct menu* load_esc_menu ( char *selected, char *title, char **labels, char **items ) {
	struct menu *menu;
	struct menu_item *item;
	struct choose_options opts;
	int i, rc, j = 0;

	menu = create_menu ( STR_EXIT, title );
        if ( ! menu )
		return NULL;

	/* Add empty line */
	ESC_MENU_ADD_ITEM( NULL, NULL );
	/* Add lines */
	for ( i = 0 ; i < TOTAL_MENU_ITEMS ; i++ ) {
		ESC_MENU_ADD_ITEM ( labels[i], items[i] );
	}

	opts.select = "selected || exit";
	if ( ( rc = show_menu ( menu, 0, opts.select , &item ) ) != 0 ) {
		selected[0] = '\0';
		return menu;
	}

	snprintf ( selected, SELECTED_STR_MAX_LEN, "%s", item->label );
	return menu;
}

static int open_esc_menu () {
	struct menu *menu;
	char selected[SELECTED_STR_MAX_LEN] = { 0 };
	char *esc_labels[TOTAL_MENU_ITEMS] = { STR_SAVE, STR_DISCARD, STR_BACK };
	char *esc_items[TOTAL_MENU_ITEMS] = { STR_SAVE_CONF, STR_ESC_DISCARD_CHANGES, STR_GO_BACK };
	char *shell_labels[TOTAL_MENU_ITEMS] = { STR_NO, STR_YES, NULL };
	char *shell_items[TOTAL_MENU_ITEMS] = { STR_CONTINUE_BOOT, STR_EXIT_TO_SHELL, NULL };

	menu = load_esc_menu ( selected, STR_EXIT, esc_labels, esc_items );
	destroy_menu ( menu );

	if ( strcmp ( selected, STR_SAVE ) == 0 )
		store_settings_to_flash ();
	else if ( ( strcmp ( selected, STR_BACK ) == 0 ) || ( selected[0] == 0 ) )
		return BACK;

	menu = load_esc_menu ( selected, STR_EXIT_TO_SHELL_TITLE, shell_labels, shell_items );
	destroy_menu ( menu );

	if ( strcmp ( selected, STR_YES ) == 0 )
		return SHELL;

	return EXIT;
}

static int main_loop ( struct settings *settings ) {
	struct setting_widget widget;
	struct extended_setting *ext;
	unsigned int next;
	int redraw = 1;
	int key, rc, type, move;

	/* Print initial screen content */
	color_set ( CPAIR_NORMAL, NULL );
	memset ( &widget, 0, sizeof ( widget ) );
	init_widget ( &widget, settings );

	while ( 1 ) {

		/* Redraw rows if necessary */
		if ( redraw ) {
			draw_title_row ( &widget );
			draw_info_row ( &widget );
			draw_instruction_row ( &widget );
			color_set ( ( widget.row.editing ?
				      CPAIR_EDIT : CPAIR_SELECT ), NULL );
			draw_setting_row ( &widget );
			color_set ( CPAIR_NORMAL, NULL );
			curs_set ( widget.row.editing );
			redraw = 0;
		}

		if ( widget.row.editing ) {

			/* Sanity check */
			assert ( widget.row.setting.name != NULL );

			/* Redraw edit box */
			color_set ( CPAIR_EDIT, NULL );
			draw_editbox ( &widget.row.editbox );
			color_set ( CPAIR_NORMAL, NULL );

			/* Process keypress */
			key = edit_setting ( &widget, getkey ( 0 ) );
			switch ( key ) {
			case CR:
			case LF:
				if ( ( rc = save_setting ( &widget ) ) != 0 )
					alert ( " Invalid input " );
				/* Fall through */
			case CTRL_C:
				select_setting_row ( &widget, widget.current );
				redraw = 1;
				break;
			default:
				/* Do nothing */
				break;
			}

		} else {

			/* Process keypress */
			key = getkey ( 0 );
			move = 0;
			switch ( key ) {
			case KEY_UP:
				move = -1;
				break;
			case KEY_DOWN:
				move = +1;
				break;
			case KEY_PPAGE:
				move = ( widget.first_visible -
					 widget.current - 1 );
				break;
			case KEY_NPAGE:
				move = ( widget.first_visible - widget.current
					 + SETTINGS_LIST_ROWS );
				break;
			case KEY_HOME:
				move = -widget.num_rows;
				break;
			case KEY_END:
				move = +widget.num_rows;
				break;
			case CTRL_D:
				if ( widget.row.settings ) {
					break;
				}
				ext = find_extended ( &widget.row.setting );
				GET_EXTENDED_TYPE( ext, widget.settings );
				if ( ( type == OPTION ) || ( type == INPUT ) ) {
					if ( ( rc = delete_setting ( widget.settings,
								&widget.row.setting ) ) != 0 ) {
						alert ( " %s ", strerror ( rc ) );
					}
					select_setting_row ( &widget, widget.current );
					redraw = 1;
				}
				break;
			case CTRL_R:
				if ( widget.row.settings && widget.settings->default_scope == &main_scope ) {
                                        restore_all_defaults ();
                                        init_widget ( &widget, widget.settings );
                                        redraw = 1;
                                } else if ( widget.row.settings && widget.settings->default_scope == &port_scope ) {
					restore_defaults ( widget.settings );
					init_widget ( &widget, widget.settings );
                                        redraw = 1;
				}
				break;
			case CTRL_S:
				endwin();
				store_settings_to_flash();
				destroy_driver_settings ();
				return 0;
			case CR:
			case LF:
				if ( widget.row.settings ) {
					init_widget ( &widget,
						      widget.row.settings );
					redraw = 1;
				} else {
					ext = find_extended (&widget.row.setting);
					GET_EXTENDED_TYPE( ext, widget.settings );
					if ( type == INPUT ) {
						edit_setting ( &widget, key );
                                                redraw = 1;
                                        } else if ( type == OPTION ) {
						change_setting ( &widget );
						redraw = 1;
					} else {
						/* type is LABEL - do nothing */
					}
				}
				break;
			case ESC:
				if ( widget.settings->default_scope != &main_scope ) {
					init_widget ( &widget, widget.settings->parent);
					redraw = 1;
				} else {
					endwin();
					rc = open_esc_menu();
					if ( rc == BACK ) {
						initscr();
						start_color();
						color_set ( CPAIR_NORMAL, NULL );
						curs_set ( 0 );
						erase();
						color_set ( CPAIR_NORMAL, NULL );
					        memset ( &widget, 0, sizeof ( widget ) );
        					init_widget ( &widget, settings );
						redraw = 1;
					} else if ( rc == SHELL ) {
						destroy_driver_settings();
						shell();
						return 0;
					} else { /* rc == EXIT */
						destroy_driver_settings();
						return 0;
					}
				}
				break;
			default:
				if ( widget.row.setting.name ) {
					ext = find_extended (&widget.row.setting);
					GET_EXTENDED_TYPE( ext, widget.settings );
					if ( type == INPUT ) {
						edit_setting ( &widget, key );
						redraw = 1;
					}
				}
				break;
			}
			if ( move ) {
				next = ( widget.current + move );
				if ( ( int ) next < 0 )
					next = 0;
				if ( next >= widget.num_rows )
					next = ( widget.num_rows - 1 );
				if ( next != widget.current ) {
					draw_setting_row ( &widget );
					redraw = 1;
					reveal_setting_row ( &widget, next );
					select_setting_row ( &widget, next );
				}
			}
		}
	}
}

int boot_menu_ui () {
	int rc;

	/* Check if there are no driver settings registered,
	* if so then dont show the menu
	*/
	if ( ( ! nv_settings_root ) || list_empty ( &nv_settings_root->settings.children )
		|| list_empty ( &driver_settings_list ) )
		return 0;

	initscr();
	start_color();
	color_set ( CPAIR_NORMAL, NULL );
	curs_set ( 0 );
	erase();

	enable_driver_settings_store ();
	rc = main_loop ( &nv_settings_root->settings );

	return rc;
}

void boot_menu_set_mode ( uint8_t is_rw ) {
	boot_menu_rw = is_rw;
}

uint8_t boot_menu_get_mode () {
	return boot_menu_rw;
}
