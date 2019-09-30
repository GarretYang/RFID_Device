#ifndef GLOBAL_H
#define GLOBAL_H

#define NONE 0;
#define MOTOR_ON 1
#define HAS_TAG 2
#define LENGTH 4

typedef struct Info{
  char* id;
  char* name;
} Info;

// locked = 0
// unlocked = 1
int LOCKED = 0;
int INDEX = -1;

char tag0[] = {0xBD, 0x76, 0x5E, 0x7B, 0xEE};
char tag1[] = {0xF1, 0xC7, 0x53, 0x73, 0x16};
char tag2[] = {0x13, 0x0F, 0x53, 0x73, 0x3C};
char tag3[] = {0x34, 0x3F, 0x4A, 0x73, 0x32};

char* TAGS[] = {
  tag0, tag1, tag2, tag3
  };
Info INFO[] = {{"1", "Ariel"}, {"2", "Garret"}, {"3", "Michael"}, {"4", "Nikola"}};



#endif