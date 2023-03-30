// MASTER -->> SLAVE COMMANDS
#define TOROFF      0
#define PING        (unsigned char ) 1
#define GOPOS       2
#define ASSVEL      3
#define ASSACCEL    4
#define READPOS     5
#define READVEL     6
#define READCUR     7
#define READTOR     8
#define READSTATE   9

// SLAVE STATES
#define DIS         0 
#define READY       1
#define SETGOAL     2
#define ASSIGN      3
#define EXEC        4     