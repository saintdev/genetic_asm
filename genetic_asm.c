#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <string.h>

#define NUMREGS 16
#define MAX_INSTR 500
#define NUM_PROGRAMS 20

#define TRIES 1000000
#define ITERATIONS 1000000
#define WEIGHT 20
#define STARTTEMP 100

static uint8_t levels[4*4];
static uint8_t coeffs[4*4];

static uint8_t srcregisters[NUMREGS][8];
static uint8_t resultregisters[8][8];
static uint8_t registers[NUMREGS][8];
static uint8_t counter[65];

typedef struct program {
    int length;
    uint8_t instructions[MAX_INSTR][4];
} program_t;

#define ZIG(i,y,x) levels[i] = coeffs[x*8+y];

void init_levels_8x8()
{
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,4,0) ZIG(11,3,1)
    ZIG(12,2,2) ZIG(13,1,3) ZIG(14,0,4) ZIG(15,0,5)
    ZIG(16,1,4) ZIG(17,2,3) ZIG(18,3,2) ZIG(19,4,1)
    ZIG(20,5,0) ZIG(21,6,0) ZIG(22,5,1) ZIG(23,4,2)
    ZIG(24,3,3) ZIG(25,2,4) ZIG(26,1,5) ZIG(27,0,6)
    ZIG(28,0,7) ZIG(29,1,6) ZIG(30,2,5) ZIG(31,3,4)
    ZIG(32,4,3) ZIG(33,5,2) ZIG(34,6,1) ZIG(35,7,0)
    ZIG(36,7,1) ZIG(37,6,2) ZIG(38,5,3) ZIG(39,4,4)
    ZIG(40,3,5) ZIG(41,2,6) ZIG(42,1,7) ZIG(43,2,7)
    ZIG(44,3,6) ZIG(45,4,5) ZIG(46,5,4) ZIG(47,6,3)
    ZIG(48,7,2) ZIG(49,7,3) ZIG(50,6,4) ZIG(51,5,5)
    ZIG(52,4,6) ZIG(53,3,7) ZIG(54,4,7) ZIG(55,5,6)
    ZIG(56,6,5) ZIG(57,7,4) ZIG(58,7,5) ZIG(59,6,6)
    ZIG(60,5,7) ZIG(61,6,7) ZIG(62,7,6) ZIG(63,7,7)
}

#undef ZIG
#define ZIG(i,y,x) levels[i] = coeffs[x*4+y];

void init_levels_4x4()
{
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            coeffs[i*4+j] = rand() % UINT8_MAX;
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,3,1) ZIG(11,2,2)
    ZIG(12,1,3) ZIG(13,2,3) ZIG(14,3,2) ZIG(15,3,3)
}

void init_levels()
{
    init_levels_4x4();
}

enum instructions
{
    PUNPCKLWD   = 0,
    PUNPCKHWD   = 1,
    PUNCPKLDQ   = 2,
    PUNPCKHDQ   = 3,
    PUNPCKLQDQ  = 4,
    PUNPCKHQDQ  = 5,
    PSLLDQ      = 6,
    PSRLDQ      = 7,
    PSLLQ       = 8,
    PSRLQ       = 9,
    PSLLD       = 10,
    PSRLD       = 11,
    PSHUFLW     = 12,
    PSHUFHW     = 13,
    NUM_INSTR   = 14
};


static const int allowedshuf[24] = { (0<<6)+(1<<4)+(2<<2)+(3<<0),
                                     (0<<6)+(1<<4)+(3<<2)+(2<<0),
                                     (0<<6)+(2<<4)+(3<<2)+(1<<0),
                                     (0<<6)+(2<<4)+(1<<2)+(3<<0),
                                     (0<<6)+(3<<4)+(2<<2)+(1<<0),
                                     (0<<6)+(3<<4)+(1<<2)+(2<<0),

                                     (1<<6)+(0<<4)+(2<<2)+(3<<0),
                                     (1<<6)+(0<<4)+(3<<2)+(2<<0),
                                     (1<<6)+(2<<4)+(3<<2)+(0<<0),
                                     (1<<6)+(2<<4)+(0<<2)+(3<<0),
                                     (1<<6)+(3<<4)+(2<<2)+(0<<0),
                                     (1<<6)+(3<<4)+(0<<2)+(2<<0),

                                     (2<<6)+(1<<4)+(0<<2)+(3<<0),
                                     (2<<6)+(1<<4)+(3<<2)+(0<<0),
                                     (2<<6)+(0<<4)+(3<<2)+(1<<0),
                                     (2<<6)+(0<<4)+(1<<2)+(3<<0),
                                     (2<<6)+(3<<4)+(0<<2)+(1<<0),
                                     (2<<6)+(3<<4)+(1<<2)+(0<<0),

                                     (3<<6)+(1<<4)+(2<<2)+(0<<0),
                                     (3<<6)+(1<<4)+(0<<2)+(2<<0),
                                     (3<<6)+(2<<4)+(0<<2)+(1<<0),
                                     (3<<6)+(2<<4)+(1<<2)+(0<<0),
                                     (3<<6)+(0<<4)+(2<<2)+(1<<0),
                                     (3<<6)+(0<<4)+(1<<2)+(2<<0)};

void print_instructions( uint8_t (*instructions)[4], int num_instructions )
{
    int i;
    for( i = 0; i < num_instructions; i++ )
    {
        uint8_t *instr = instructions[i];
        if( instr[0] != PSHUFLW && instr[0] != PSHUFHW && instr[1] != instr[3] ) printf("movdqa m%d, m%d\n", instr[3], instr[1] );
        switch( instr[0] )
        {
            case PUNPCKLWD:
                printf( "punpcklwd" );
                break;
            case PUNPCKHWD:
                printf( "punpckhwd" );
                break;
            case PUNCPKLDQ:
                printf( "punpckldq" );
                break;
            case PUNPCKHDQ:
                printf( "punpckhdq" );
                break;
            case PUNPCKLQDQ:
                printf( "punpcklqdq" );
                break;
            case PUNPCKHQDQ:
                printf( "punpckhqdq" );
                break;
            case PSLLDQ:
                printf( "pslldq" );
                break;
            case PSRLDQ:
                printf( "psrldq" );
                break;
            case PSLLQ:
                printf( "psllq" );
                break;
            case PSRLQ:
                printf( "psrlq" );
                break;
            case PSLLD:
                printf( "pslld" );
                break;
            case PSRLD:
                printf( "psrld" );
                break;
            case PSHUFLW:
                printf( "pshuflw" );
                break;
            case PSHUFHW:
                printf( "pshufhw" );
                break;
            default:
                fprintf( stderr, "Error: unsupported instruction!\n");
                assert(0);
        }
        if(instr[0] < PSLLDQ ) {
            printf(" m%d, ", instr[1], instr[2] );
            if (instr[2] < NUMREGS)
                printf("m%d", instr[2]);
            else
                printf("0x%x", allowedshuf[instr[2] - NUMREGS]);
        } else if(instr[0] < PSHUFLW ) {
            printf(" m%d, ", instr[1]);
            if (instr[2] < NUMREGS)
                printf("m%d", instr[2]);
            else
                printf("%d", instr[2] - NUMREGS);
        }
        else                    printf(" m%d, m%d, 0x%x", instr[1], instr[2], instr[3] );
        printf("\n");
    }
}

void execute_instruction( uint8_t *instr )
{
    uint8_t *input1 = registers[instr[1]];
    uint8_t *input2 = registers[instr[2]];
    uint8_t *output = registers[instr[3]];
    uint8_t temp[8];
    int i;
    int imm = instr[2];

    switch( instr[0] )
    {
        case PUNPCKLWD:
            temp[0] = input1[0];
            temp[2] = input1[1];
            temp[4] = input1[2];
            temp[6] = input1[3];
            temp[1] = input2[0];
            temp[3] = input2[1];
            temp[5] = input2[2];
            temp[7] = input2[3];
            break;
        case PUNPCKHWD:
            temp[0] = input1[4];
            temp[2] = input1[5];
            temp[4] = input1[6];
            temp[6] = input1[7];
            temp[1] = input2[4];
            temp[3] = input2[5];
            temp[5] = input2[6];
            temp[7] = input2[7];
            break;
        case PUNCPKLDQ:
            temp[0] = input1[0];
            temp[1] = input1[1];
            temp[4] = input1[2];
            temp[5] = input1[3];
            temp[2] = input2[0];
            temp[3] = input2[1];
            temp[6] = input2[2];
            temp[7] = input2[3];
            break;
        case PUNPCKHDQ:
            temp[0] = input1[4];
            temp[1] = input1[5];
            temp[4] = input1[6];
            temp[5] = input1[7];
            temp[2] = input2[4];
            temp[3] = input2[5];
            temp[6] = input2[6];
            temp[7] = input2[7];
            break;
        case PUNPCKLQDQ:
            temp[0] = input1[0];
            temp[1] = input1[1];
            temp[2] = input1[2];
            temp[3] = input1[3];
            temp[4] = input2[0];
            temp[5] = input2[1];
            temp[6] = input2[2];
            temp[7] = input2[3];
            break;
        case PUNPCKHQDQ:
            temp[0] = input1[4];
            temp[1] = input1[5];
            temp[2] = input1[6];
            temp[3] = input1[7];
            temp[4] = input2[4];
            temp[5] = input2[5];
            temp[6] = input2[6];
            temp[7] = input2[7];
            break;
        case PSLLDQ:
            for( i = 0; i < imm; i++ )
                temp[i] = input1[i+imm];
            for( i = imm; i < 8; i++ )
                temp[i] = 64;
            break;
        case PSRLDQ:
            for( i = 0; i < imm; i++ )
                temp[i] = 64;
            for( i = imm; i < 8; i++ )
                temp[i] = input1[i-imm];
            break;
        case PSLLQ:
            if( imm == 1 )
            {
                temp[0] = input1[1];
                temp[1] = input1[2];
                temp[2] = input1[3];
                temp[3] = 64;
                temp[4] = input1[5];
                temp[5] = input1[6];
                temp[6] = input1[7];
                temp[7] = 64;
            }
            else if( imm == 2 )
            {
                temp[0] = input1[2];
                temp[1] = input1[3];
                temp[2] = 64;
                temp[3] = 64;
                temp[4] = input1[6];
                temp[5] = input1[7];
                temp[6] = 64;
                temp[7] = 64;
            }
            else if( imm == 3 )
            {
                temp[0] = input1[3];
                temp[1] = 64;
                temp[2] = 64;
                temp[3] = 64;
                temp[4] = input1[7];
                temp[5] = 64;
                temp[6] = 64;
                temp[7] = 64;
            }
            else
                assert(0);
            break;
        case PSRLQ:
            if( imm == 1 )
            {
                temp[0] = 64;
                temp[1] = input1[0];
                temp[2] = input1[1];
                temp[3] = input1[2];
                temp[4] = 64;
                temp[5] = input1[4];
                temp[6] = input1[5];
                temp[7] = input1[6];
            }
            else if( imm == 2 )
            {
                temp[0] = 64;
                temp[1] = 64;
                temp[2] = input1[0];
                temp[3] = input1[1];
                temp[4] = 64;
                temp[5] = 64;
                temp[6] = input1[0];
                temp[7] = input1[1];
            }
            else if( imm == 3 )
            {
                temp[0] = 64;
                temp[1] = 64;
                temp[2] = 64;
                temp[3] = input1[0];
                temp[4] = 64;
                temp[5] = 64;
                temp[6] = 64;
                temp[7] = input1[0];
            }
            else
                assert(0);
            break;
        case PSLLD:
            assert( imm == 1 );
            temp[0] = input1[1];
            temp[1] = 64;
            temp[2] = input1[3];
            temp[3] = 64;
            temp[4] = input1[5];
            temp[5] = 64;
            temp[6] = input1[7];
            temp[7] = 64;
            break;
        case PSRLD:
            assert( imm == 1 );
            temp[0] = 64;
            temp[1] = input1[0];
            temp[2] = 64;
            temp[3] = input1[2];
            temp[4] = 64;
            temp[5] = input2[4];
            temp[6] = 64;
            temp[7] = input2[6];
            break;
        case PSHUFLW:
            temp[0] = input1[imm&0x3]; imm >>= 2;
            temp[1] = input1[imm&0x3]; imm >>= 2;
            temp[2] = input1[imm&0x3]; imm >>= 2;
            temp[3] = input1[imm&0x3];
            for( i = 4; i < 8; i++ )
                temp[i] = input1[i];
            break;
        case PSHUFHW:
            temp[4] = input1[4+(imm&0x3)]; imm >>= 2;
            temp[5] = input1[4+(imm&0x3)]; imm >>= 2;
            temp[6] = input1[4+(imm&0x3)]; imm >>= 2;
            temp[7] = input1[4+(imm&0x3)];
            for( i = 0; i < 4; i++ )
                temp[i] = input1[i];
            break;
        default:
            fprintf( stderr, "Error: unsupported instruction %d %d %d %d!\n", instr[0], instr[1], instr[2], instr[3]);
            assert(0);
    }
    for( i = 0; i < 8; i++ )
    {
        counter[output[i]]--;
        counter[temp[i]]++;
    }
    memcpy( output, temp, 8*sizeof(uint8_t) );
}

void init_resultregisters()
{
    int r, i;
    for(r=0;r<8;r++)
        for(i=0;i<8;i++)
            resultregisters[r][i] = levels[i+r*8];
}

void init_srcregisters()
{
    int r, i;
    for(r = 0; r < 2; r++)
        memcpy(srcregisters[r], &coeffs[r*8], sizeof(uint8_t) * 8);
    for(; r < NUMREGS; r++)
        memset(srcregisters[r], 0, sizeof(srcregisters[r][0]) * 8);
}

void init_registers()
{
    memcpy( registers, srcregisters, sizeof(srcregisters) );
}

void init_programs(program_t *programs)
{
    for(int i = 0; i < NUM_PROGRAMS; i++) {
        program_t *program = &programs[i];
        program->length = (rand() % (MAX_INSTR - 1)) + 1;
        for(int j = 0; j < program->length; j++) {
            int instr = rand() % NUM_INSTR;
            int output = rand() % NUMREGS;
            int input1, input2 = 0;

            /* FIXME: This should be completely random, instead of the guided randomnes we have below.
             * This may help generate more valid code, however.
             */
            if( instr < PSLLDQ )
                input1 = rand() % (NUMREGS + 24);
            else if( instr < PSLLQ )
                input1 = rand() % 16;
            else if( instr < PSLLD )
                input1 = rand() % (NUMREGS + 64);
            else if( instr < PSHUFLW )
                input1 = rand() % (NUMREGS + 32);
            else {
                input1 = rand() % NUMREGS;
                input2 = allowedshuf[rand() % 24];
            }

            program->instructions[j][0] = instr;
            program->instructions[j][1] = output;
            program->instructions[j][2] = input1;
            program->instructions[j][3] = input2;
        }
    }
}

int run_program( uint8_t (*instructions)[4], int num_instructions, int debug )
{
    int i,r,j;
    init_registers();
    memset( counter, 1, sizeof( counter ) );
    if( debug )
    {
        printf("targetregs: \n");
        for(r=0;r<8;r++)
        {
            for(j=0;j<8;j++)
                printf("%d ",resultregisters[r][j]);
            printf("\n");
        }
    }
    for( i = 0; i < num_instructions; i++ )
    {
        if(debug)
        {
            printf("regs: \n");
            for(r=0;r<NUMREGS;r++)
            {
                for(j=0;j<8;j++)
                    printf("%d ",registers[r][j]);
                printf("\n");
            }
        }
        execute_instruction( instructions[i] );
        //for( i = 0; i < 64; i++ )
          //  if( !counter[i] ) return 0;
    }
    return 1;
}

//#define CHECK_LOC if( i >= 2 && i <= 5 ) continue;
#define CHECK_LOC if( 0 ) continue;

int result_cost( int *correct )
{
    //static const int errorcosts[9] = { 0, 7, 14, 21, 27, 32, 36, 39, 40 };
    static const int errorcosts[9] = { 0, 80, 110, 121, 127, 132, 136, 139, 140 };
    static const int weight[8] = { 2, 3, 4, 5, 5, 4, 3, 2 };
    int sumerror = 0;
    int ssderror = 0;
    int i,j;
    for( i = 0; i < 8; i++ )
    {
        CHECK_LOC
        int regerror = 0;
        for( j = 0; j < 8; j++ )
            regerror += registers[i][j] != resultregisters[i][j];
        sumerror += regerror;
        //ssderror += regerror*regerror;
        ssderror += errorcosts[regerror] * weight[i];
    }
    *correct = 64 - sumerror;
    return ssderror;
}

void result_cost_breakdown()
{
    int sumerror = 0;
    int i,j;
    printf("Regerror breakdown: ");
    for( i = 0; i < 8; i++ )
    {
        CHECK_LOC
        int regerror = 0;
        for( j = 0; j < 8; j++ )
            regerror += registers[i][j] != resultregisters[i][j];
        printf("%d ",regerror);
    }
    printf("\n");
}

void instruction_delete( uint8_t (*instructions)[4], int loc, int numinstructions )
{
    int i;
    for( i = loc; i < numinstructions-1; i++ )
    {
        instructions[i][0] = instructions[i+1][0];
        instructions[i][1] = instructions[i+1][1];
        instructions[i][2] = instructions[i+1][2];
        instructions[i][3] = instructions[i+1][3];
    }
}

void instruction_shift( uint8_t (*instructions)[4], int loc, int numinstructions )
{
    int i;
    for( i = numinstructions; i > loc; i-- )
    {
        instructions[i][0] = instructions[i-1][0];
        instructions[i][1] = instructions[i-1][1];
        instructions[i][2] = instructions[i-1][2];
        instructions[i][3] = instructions[i-1][3];
    }
}

#define MAXCHANGES 16

int generate_algorithm( uint8_t (*instructions)[4], uint8_t (*srcinstructions)[4], int numinstructions )
{

    memcpy( instructions, srcinstructions, numinstructions * sizeof(uint8_t) * 4 );
    int numchanges = rand() % MAXCHANGES;
    int i;
    for( i = 0; i < numchanges; i++ )
    {
        int change = rand() % 4;
        int instr = rand() % NUM_INSTR;
        int input1 = rand() % NUMREGS;
        int input2;
        if( instr < PSLLDQ )
            input2 = rand() % NUMREGS;
        else if( instr < PSLLQ )
            input2 = (rand() % 7) + 1;
        else if( instr < PSLLD )
            input2 = (rand() % 3) + 1;
        else if( instr < PSHUFLW )
            input2 = 1;
        else
            input2 = allowedshuf[rand() % 24];

        int output = rand() % NUMREGS;
        int loc = rand() & 1 ? rand() % (numinstructions+1) : numinstructions;
        if(loc == numinstructions) change=1;
        switch( change )
        {
            case 0: //delete instruction
            if( numinstructions == 0 ) continue;
            instruction_delete( instructions, loc, numinstructions );
            numinstructions--;
            break;
            case 1:
            case 2: //add instruction
            if( numinstructions == MAX_INSTR ) continue;
            instruction_shift( instructions, loc, numinstructions );
            numinstructions++;
            instructions[loc][0] = instr;
            instructions[loc][1] = input1;
            instructions[loc][2] = input2;
            instructions[loc][3] = output;
            break;
            case 3: //change instruction
            instructions[loc][0] = instr;
            instructions[loc][1] = input1;
            instructions[loc][2] = input2;
            instructions[loc][3] = output;
            break;
        }
    }
}

int instruction_cost( uint8_t (*instructions)[4], int numinstructions )
{
    int i;
    int cost = 0;
    for( i = 0; i < numinstructions; i++ )
    {
        cost++;
        if( instructions[i][0] != PSHUFLW && instructions[i][0] != PSHUFHW && instructions[i][1] != instructions[i][3] ) cost++;
    }
    return cost;
}

int main()
{
    program_t programs[NUM_PROGRAMS];
//     static uint8_t binstructions[MAX_INSTR][4] = {0};
//     int num_binstructions = 0;
//     int bcost = 1<<30;
//     int binstruction_cost = 0;
//     int bcorrect = 0;
    int t, i;

    int ltime;
//     int temperature = STARTTEMP;
//     int gototemp = STARTTEMP;
//     int iterations_since_change = 0;
//     int oldbestbcost = 1<<30;
//     int bestbcost = 1<<30;

    /* get the current calendar time */
    ltime = time(NULL);
    srand(ltime);

    init_levels();
    init_srcregisters();
    init_resultregisters();
    init_programs(programs);

    for(i = 0; i < NUM_PROGRAMS; i++) {
        print_instructions(programs[i].instructions, programs[i].length);
        printf("\n");
    }
#if 0
    for( i = 0; i < ITERATIONS; i++ )
    {
        int prevcost = bcost;
        for( t = 0; t < TRIES; t++ )
        {
            num_instructions = generate_algorithm( instructions, binstructions, num_binstructions );
            int success = run_program( instructions, num_instructions, 0 );
            if( !success ) continue;
            int correct;
            int output_cost = result_cost( &correct );
            //if( output_cost > bcost ) continue;
            int icost = instruction_cost( instructions, num_instructions );
            int cost = output_cost * WEIGHT + icost;
            int weight = temperature ? rand() % temperature : 0;
            if( cost < bcost + weight )
            {
                bcost = cost;
                bcorrect = correct;
                binstruction_cost = icost;
                memcpy( binstructions, instructions, num_instructions * sizeof(uint8_t) * 4 );
                num_binstructions = num_instructions;
                if( cost < bestbcost ) bestbcost = cost;
            }
        }
        printf("Correct Outputs: %d, Instructions: %d\n",bcorrect,binstruction_cost);
        printf("Temperature: %d Iterations Since Change: %d Gototemp: %d\n",temperature,iterations_since_change, gototemp);
        run_program( binstructions, num_binstructions, 0 );
        result_cost_breakdown();
        print_instructions( binstructions, num_binstructions );
        if( temperature ) temperature--;
        if( !temperature && prevcost == bcost ) iterations_since_change++;
        else iterations_since_change = 0;
        if( iterations_since_change == 100 )
        {
            temperature = gototemp;
            if( bestbcost == oldbestbcost ) gototemp += 100;
            else if( gototemp > 100 ) gototemp -= 100;
            oldbestbcost = bestbcost;
        }
    }
#endif
    return 0;
}