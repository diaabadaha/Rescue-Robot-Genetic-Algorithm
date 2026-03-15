#include "../include/menu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

// Helper: Create default menu if file loading fails
// Helper: Create default menu if file loading fails
static MenuLayout *create_default_menu(void)
{
    MenuLayout *layout = malloc(sizeof(MenuLayout));
    if (!layout)
        return NULL;

    layout->count = 6;

    layout->entries[0].option = MENU_RUN_GA;
    strcpy(layout->entries[0].label, "Run Genetic Algorithm");

    layout->entries[1].option = MENU_LOAD_MAP;
    strcpy(layout->entries[1].label, "Load and Display Map");

    layout->entries[2].option = MENU_VIEW_CONFIG;
    strcpy(layout->entries[2].label, "View Current Configuration");

    layout->entries[3].option = MENU_TEST_WORKERS;
    strcpy(layout->entries[3].label, "Test Worker Pool");

    layout->entries[4].option = MENU_RUN_SINGLE_GEN;
    strcpy(layout->entries[4].label, "Run Single Generation");

    layout->entries[5].option = MENU_EXIT;
    strcpy(layout->entries[5].label, "Exit");

    return layout;
}

static MenuOption parse_menu_option(const char *label)
{
    if (strstr(label, "Run Genetic Algorithm") || strstr(label, "Run GA"))
        return MENU_RUN_GA;
    if (strstr(label, "Load and Display Map"))
        return MENU_LOAD_MAP;
    if (strstr(label, "Load/Generate New Map"))
        return MENU_LOAD_GENERATE_MAP;
    if (strstr(label, "View") && strstr(label, "Config"))
        return MENU_VIEW_CONFIG;
    if (strstr(label, "Test") && strstr(label, "Worker"))
        return MENU_TEST_WORKERS;
    if (strstr(label, "Single Generation"))
        return MENU_RUN_SINGLE_GEN;
    if (strstr(label, "Performance Comparison"))
        return MENU_PERFORMANCE_COMPARISON;
    if (strstr(label, "Exit"))
        return MENU_EXIT;

    return MENU_INVALID;
}

// Load menu from file
MenuLayout *load_menu_layout(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f)
    {
        return NULL;
    }

    MenuLayout *layout = malloc(sizeof(MenuLayout));
    if (!layout)
    {
        fclose(f);
        return NULL;
    }

    layout->count = 0;

    char line[256];
    while (fgets(line, sizeof(line), f) && layout->count < 10)
    {
        // Remove newline
        line[strcspn(line, "\n")] = 0;

        // Skip empty lines
        if (strlen(line) == 0)
            continue;

        MenuOption opt = parse_menu_option(line);
        if (opt != MENU_INVALID)
        {
            layout->entries[layout->count].option = opt;
            strncpy(layout->entries[layout->count].label, line, 99);
            layout->entries[layout->count].label[99] = '\0';
            layout->count++;
        }
    }

    fclose(f);

    if (layout->count == 0)
    {
        free(layout);
        return NULL;
    }

    return layout;
}

// Display menu
void display_menu(const MenuLayout *layout)
{
    printf("\n");
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║     RESCUE ROBOT PATH OPTIMIZATION SYSTEM          ║\n");
    printf("║               Genetic Algorithm                    ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    printf("\n");

    for (int i = 0; i < layout->count; i++)
    {
        printf("  %d. %s\n", i + 1, layout->entries[i].label);
    }

    printf("\n");
}

// Get user choice
MenuOption get_user_choice(const MenuLayout *layout)
{
    int choice;
    printf("Enter your choice (1-%d): ", layout->count);
    fflush(stdout);

    if (scanf("%d", &choice) != 1)
    {
        // Clear input buffer on invalid input
        int c;
        while ((c = getchar()) != '\n' && c != EOF)
            ;
        return MENU_INVALID;
    }

    // Clear the newline left by scanf
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;

    if (choice < 1 || choice > layout->count)
    {
        return MENU_INVALID;
    }

    return layout->entries[choice - 1].option;
}

// Free menu layout
void free_menu_layout(MenuLayout *layout)
{
    if (layout)
    {
        free(layout);
    }
}

// Helper function to get menu layout (tries file first, falls back to default)
MenuLayout *get_menu_layout(const char *filename)
{
    MenuLayout *layout = load_menu_layout(filename);
    if (!layout)
    {
        printf("Using default menu layout\n");
        layout = create_default_menu();
    }
    return layout;
}