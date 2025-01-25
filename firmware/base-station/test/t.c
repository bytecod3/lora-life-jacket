#include <stdio.h>
#include <string.h>

#define ROWS 10
#define COLS 20

char split_strings[ROWS][COLS];

void decode_msg(char* msg) {
    char copied[20];

    printf("%s\n", msg);

    char* token = strtok(msg, ",");

    int index = 0;
    while (token != NULL) {
        printf("%s\n", token);
        strcpy(copied, token);
        strcpy(split_strings[index], copied);
        token = strtok(NULL, ",");
        index++;
    }

    // printf("Processed strings\n");
    // for(int i=0; i < ROWS; i++) {
    //     printf("%s\n", split_strings[i]);
    // }

    if(strcmp(split_strings[1], "SOS") == 0) {
        printf("SOS found");
    } else if(strcmp(split_strings[1], "OK") == 0) {
        printf("OK found");
    }
}

int main() {
    char f[] = "D7 C1 80 35,OK,0.00,0.00,3,0,0,0,0,0";
    decode_msg(f);

    return 0;
}