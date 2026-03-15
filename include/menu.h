#ifndef MENU_H
#define MENU_H

// Menu options enum
typedef enum
{
    MENU_RUN_GA = 1,
    MENU_LOAD_MAP = 2,
    MENU_LOAD_GENERATE_MAP = 3,
    MENU_VIEW_CONFIG = 4,
    MENU_TEST_WORKERS = 5,
    MENU_RUN_SINGLE_GEN = 6,
    MENU_PERFORMANCE_COMPARISON = 7,
    MENU_EXIT = 8,
    MENU_INVALID = -1
} MenuOption;
// Menu entry structure
typedef struct {
    MenuOption option;
    char label[100];
} MenuEntry;

// Menu layout structure (holds all entries in user-defined order)
typedef struct {
    MenuEntry entries[10];  // Max 10 menu items
    int count;              // Actual number of entries
} MenuLayout;

// Function prototypes
MenuLayout* load_menu_layout(const char* filename);
void display_menu(const MenuLayout* layout);
MenuOption get_user_choice(const MenuLayout* layout);
void free_menu_layout(MenuLayout* layout);
MenuLayout* get_menu_layout(const char* filename);  // Helper function

#endif // MENU_H