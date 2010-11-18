#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <limits.h>

#define NUMREGS 16
#define MAX_INSTR 500
#define NUM_PROGRAMS 100

#define TRIES 1000000
#define ITERATIONS 1000000
#define WEIGHT 20
#define STARTTEMP 100

#define LEN_ABSOLUTE  0
#define LEN_EFFECTIVE 1

static uint16_t levels[8*8];
static uint16_t coeffs[8*8];

static uint16_t srcregisters[NUMREGS][8];
static uint16_t resultregisters[8][8];
static uint16_t registers[NUMREGS][8];
// static uint8_t counter[65];

typedef struct instruction {
    uint8_t opcode;
    uint8_t operands[3];
    uint8_t flags;
    void (*fp)(uint16_t src[8], uint16_t dst[8], uint8_t imm);
} instruction_t;

typedef struct program {
    int length[2];  /* 0 = absolute, 1 = effective */
    int fitness;
    int cost;
    instruction_t instructions[MAX_INSTR];
    instruction_t effective[MAX_INSTR];
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
            coeffs[i*4+j] = rand() % UINT16_MAX;
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,3,1) ZIG(11,2,2)
    ZIG(12,1,3) ZIG(13,2,3) ZIG(14,3,2) ZIG(15,3,3)
}

void init_levels()
{
    init_levels_4x4();
}

enum instructions {
    PUNPCKLWD   = 0,
    PUNPCKHWD,
    PUNCPKLDQ,
    PUNPCKHDQ,
    PUNPCKLQDQ,
    PUNPCKHQDQ,
    MOVDQA,
    PSLLDQ,
    PSRLDQ,
    PSLLQ,
    PSRLQ,
    PSLLD,
    PSRLD,
    PSHUFLW,
    PSHUFHW,
    NUM_INSTR
};


static const uint8_t allowedshuf[24] = { (0<<6)+(1<<4)+(2<<2)+(3<<0),
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

void print_instruction( instruction_t *instr, int debug )
{
//         if( instr->opcode != PSHUFLW && instr->opcode != PSHUFHW && instr->operands[1] != instr->operands[0] ) printf("movdqa m%d, m%d\n", instr[3], instr[1] );
    switch( instr->opcode ) {
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
        case MOVDQA:
            printf( "movdqa" );
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
    if(instr->opcode < PSLLDQ ) {
                                        printf(" m%d, ", instr->operands[0]);
        if (instr->operands[1] < NUMREGS)
                                        printf("m%d", instr->operands[1]);
        else
                                        printf("0x%x", allowedshuf[instr->operands[1] - NUMREGS]);
    } else if(instr->opcode < PSHUFLW)  printf(" m%d, %d", instr->operands[0], instr->operands[2]);
    else                                printf(" m%d, m%d, 0x%x", instr->operands[0], instr->operands[1], instr->operands[2] );
    if (debug && instr->flags)          printf(" *");
}

void print_instructions( program_t *program, int debug )
{
    for( int i = 0; i < program->length[LEN_EFFECTIVE]; i++ ) {
        instruction_t *instr = &program->effective[i];
        print_instruction(instr, debug);
        printf("\n");
    }
}

void print_program( program_t *program, int debug )
{
    printf("length (absolute effective) = %d %d\n", program->length[LEN_ABSOLUTE], program->length[LEN_EFFECTIVE]);
    printf("fitness = %d\n", program->fitness);
    printf("cost = %d\n", program->cost);
    print_instructions(program, debug);
    printf("\n");
}

void execute_instruction( instruction_t instr )
{
    uint16_t *input1 = registers[instr.operands[1]];
//     uint8_t *input2 = registers[instr->operands[2]];
    uint16_t *output = registers[instr.operands[0]];
    uint16_t temp[8];
    int i;
    int imm = instr.operands[2];

    switch( instr.opcode )
    {
        case PUNPCKLWD:
            temp[0] = output[0];
            temp[2] = output[1];
            temp[4] = output[2];
            temp[6] = output[3];
            temp[1] = input1[0];
            temp[3] = input1[1];
            temp[5] = input1[2];
            temp[7] = input1[3];
            break;
        case PUNPCKHWD:
            temp[0] = output[4];
            temp[2] = output[5];
            temp[4] = output[6];
            temp[6] = output[7];
            temp[1] = input1[4];
            temp[3] = input1[5];
            temp[5] = input1[6];
            temp[7] = input1[7];
            break;
        case PUNCPKLDQ:
            temp[0] = output[0];
            temp[1] = output[1];
            temp[4] = output[2];
            temp[5] = output[3];
            temp[2] = input1[0];
            temp[3] = input1[1];
            temp[6] = input1[2];
            temp[7] = input1[3];
            break;
        case PUNPCKHDQ:
            temp[0] = output[4];
            temp[1] = output[5];
            temp[4] = output[6];
            temp[5] = output[7];
            temp[2] = input1[4];
            temp[3] = input1[5];
            temp[6] = input1[6];
            temp[7] = input1[7];
            break;
        case PUNPCKLQDQ:
            temp[0] = output[0];
            temp[1] = output[1];
            temp[2] = output[2];
            temp[3] = output[3];
            temp[4] = input1[0];
            temp[5] = input1[1];
            temp[6] = input1[2];
            temp[7] = input1[3];
            break;
        case PUNPCKHQDQ:
            temp[0] = output[4];
            temp[1] = output[5];
            temp[2] = output[6];
            temp[3] = output[7];
            temp[4] = input1[4];
            temp[5] = input1[5];
            temp[6] = input1[6];
            temp[7] = input1[7];
            break;
        case MOVDQA:
            temp[0] = input1[0];
            temp[1] = input1[1];
            temp[2] = input1[2];
            temp[3] = input1[3];
            temp[4] = input1[4];
            temp[5] = input1[5];
            temp[6] = input1[6];
            temp[7] = input1[7];
            break;
        case PSLLDQ:
            if (imm > 8) imm = 8;
            for( i = 0; i < 8 - imm; i++ )
                temp[i] = output[i+imm];
            for( ; i < 8; i++ )
                temp[i] = 0;
            break;
        case PSRLDQ:
            if (imm > 8) imm = 8;
            for( i = 0; i < imm; i++ )
                temp[i] = 0;
            for( ; i < 8; i++ )
                temp[i] = input1[i-imm];
            break;
        case PSLLQ:
        {
            uint64_t *in64 = (uint64_t*)output;
            if (imm > 64) imm = 64;
            in64[0] <<= imm;
            in64[1] <<= imm;
            break;
        }
        case PSRLQ:
        {
            uint64_t *in64 = (uint64_t*)output;
            if (imm > 64) imm = 64;
            in64[0] >>= imm;
            in64[1] >>= imm;
            break;
        }
        case PSLLD:
        {
            uint32_t *in32 = (uint32_t*)output;
            if (imm > 32) imm = 32;
            for(i = 0; i < 4; i++)
                in32[i] <<= imm;
            break;
        }
        case PSRLD:
        {
            uint32_t *in32 = (uint32_t*)output;
            if (imm > 32) imm = 32;
            for(i = 0; i < 4; i++)
                in32[i] >>= imm;
            break;
        }
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
            fprintf( stderr, "Error: unsupported instruction %d %d %d %d!\n", instr.opcode, instr.operands[0], instr.operands[1], instr.operands[2]);
            assert(0);
    }
//     for( i = 0; i < 8; i++ )
//     {
//         counter[output[i]]--;
//         counter[temp[i]]++;
//     }
    memcpy( output, temp, 8*sizeof(output[0]) );
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
        memcpy(srcregisters[r], &coeffs[r*8], sizeof(coeffs[0]) * 8);
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
        program->length[LEN_ABSOLUTE] = (rand() % (51)) + 5;
        for(int j = 0; j < program->length[LEN_ABSOLUTE]; j++) {
            int instr = rand() % NUM_INSTR;
            int output = rand() % NUMREGS;
            int input1 = NUMREGS, input2 = 0;
            instruction_t *instruction = &program->instructions[j];

            /* FIXME: This should be completely random, instead of the guided randomnes we have below.
             * This may help generate more valid code, however.
             */
            input1 = rand() % NUMREGS;
            if( instr < PSLLDQ )
                input2 = rand() % UINT8_MAX;
            else if( instr < PSLLQ )
                input2 = (rand() % 7) + 1;
            else if( instr < PSLLD )
                input2 = rand() % 64;
            else if( instr < PSHUFLW )
                input2 = rand() % 32;
            else {
//                 input2 = allowedshuf[rand() % 24];
                input2 = rand() % UINT8_MAX;
            }

            instruction->opcode = instr;
            instruction->operands[0] = output;
            instruction->operands[1] = input1;
            instruction->operands[2] = input2;
        }
    }
}

void effective_program(program_t *prog)
{
    uint8_t reg_eff[NUMREGS] = {0};
    int i = prog->length[LEN_ABSOLUTE] - 1;
    int j;

    reg_eff[0] = 1;
    reg_eff[1] = 1;
    /* We want our results in r0-r7. For 4x4 DCT, we only need two result registers. */
    while(i >= 0 && (prog->instructions[i].operands[0] != 0 && prog->instructions[i].operands[0] != 1)) {
        prog->instructions[i].flags = 0;
        i--;
    }
    for( ; i >= 0; i--) {
        instruction_t *instr = &prog->instructions[i];

        instr->flags = 0;
        for (j = 0; j < NUMREGS; j++)
            if (reg_eff[j] && instr->operands[0] == j)
                instr->flags = 1;

        if (!instr->flags)
            continue;
        if (instr->opcode >= PSHUFLW || instr->opcode == MOVDQA)
            reg_eff[instr->operands[0]] = 0;
        if (instr->operands[1] < NUMREGS &&
            (instr->opcode < PSLLDQ || instr->opcode > PSRLD))
            reg_eff[instr->operands[1]] = 1;
    }

    for(j = i = 0; i < prog->length[LEN_ABSOLUTE]; i++) {
        if (!prog->instructions[i].flags)
            continue;
        prog->instructions[i].flags = 0;
        prog->effective[j++] = prog->instructions[i];
    }
    prog->length[LEN_EFFECTIVE] = j;
}

int run_program( program_t *program, int debug )
{
    int i,r,j;

    init_registers();
//     memset( counter, 1, sizeof( counter ) );
    if( debug ) {
//         printf("sourceregs: \n");
//         for(r=0; r<8; r++) {
//             for(j=0; j<8; j++)
//                 printf("%d ", srcregisters[r][j]);
//             printf("\n");
//         }
        printf("targetregs: \n");
        for(r=0;r<8;r++) {
            for(j=0;j<8;j++)
                printf("%d ",resultregisters[r][j]);
            printf("\n");
        }
    }
    for( i = 0; i < program->length[LEN_EFFECTIVE]; i++ ) {
/*        if(debug)
        {
            printf("regs: \n");
            for(r=0;r<NUMREGS;r++)
            {
                for(j=0;j<8;j++)
                    printf("%d ",registers[r][j]);
                printf("\n");
            }
        }
*/
        execute_instruction( program->effective[i] );
        //for( i = 0; i < 64; i++ )
          //  if( !counter[i] ) return 0;
    }
    if(debug) {
        printf("resultregs: \n");
        for(r=0;r<NUMREGS;r++) {
            for(j=0;j<8;j++)
                printf("%d ",registers[r][j]);
            printf("\n");
        }
    }
    return 1;
}

//#define CHECK_LOC if( i >= 2 && i <= 5 ) continue;
#define CHECK_LOC if( 0 ) continue;

void result_fitness( program_t *prog )
{
    //static const int errorcosts[9] = { 0, 7, 14, 21, 27, 32, 36, 39, 40 };
//     static const int errorcosts[9] = { 0, 80, 110, 121, 127, 132, 136, 139, 140 };
//     static const int weight[8] = { 2, 3, 4, 5, 5, 4, 3, 2 };
    int sumerror = 0;
//     int ssderror = 0;

    for( int r = 0; r < 2; r++ ) {
        int regerror = 0;
        for(int i = 0; i < 8; i++ )
            regerror += registers[r][i] != resultregisters[r][i];
        sumerror += regerror;
//         ssderror += regerror*regerror;
//         ssderror += errorcosts[regerror] * weight[i];
    }

    prog->fitness = sumerror*sumerror;
}

void result_cost( program_t *prog )
{
    /* TODO: Use instruction latency/thouroghput */
    if (!(prog->cost = prog->length[LEN_EFFECTIVE]))
        prog->cost = INT_MAX;
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

void mutate_program( program_t *prog, float probabilities[3] )
{
    int p = rand();
    int ins_idx = rand() % prog->length[LEN_ABSOLUTE];
    instruction_t *instr = &prog->instructions[ins_idx];
    if(p < RAND_MAX * probabilities[0])                                 /* Modify an instruction */
        instr->opcode = rand() % NUM_INSTR;
    else if (p < RAND_MAX * (probabilities[0] + probabilities[1])) {    /* Modify a regester */
        if (rand() < RAND_MAX / 2)
            instr->operands[0] = rand() % NUMREGS;
        else
            instr->operands[1] = rand() % NUMREGS;
    } else                                                              /* Modify a constant */
        instr->operands[2] = rand() % UINT8_MAX;

    /* Invalidate existing fitness */
    prog->fitness = INT_MAX;
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

void run_tournament( program_t *programs, program_t *winner, int size)
{
    int contestants[NUM_PROGRAMS];
    int shift = 0;

    contestants[0] = NUM_PROGRAMS;
    for(int i = 0; i < size; i++) {
        contestants[i] = rand() % NUM_PROGRAMS;
        for(int j = 0; j < i; j++) {
            while(contestants[i] == contestants[j])
                contestants[i] = rand() % NUM_PROGRAMS;
        }
    }
    do {
//         printf("size = %d\n", size);
        for(int i = 0; i < size-1; i += (2 << shift)) {
            program_t *a = &programs[contestants[i]];
            program_t *b = &programs[contestants[i + (1<<shift)]];
            if (a->fitness < b->fitness)
                contestants[i] = contestants[i+(1<<shift)];
        }
        shift++;
    } while(size >> shift > 1);
    memcpy(winner, &programs[contestants[0]], sizeof(*winner));
}

void crossover( program_t *parents, int delta_length, int delta_pos )
{
    program_t temp[2];
    /* FIXME: Respect delta_pos, delta_length */
    int point[2];
    int length[2];

    for(int i = 0; i < 2; i++) {
        point[i] = rand() % parents[i].length[LEN_ABSOLUTE];
        length[i] = (rand() % (parents[i].length[LEN_ABSOLUTE] - point[i])) + 1;
    }

    for(int i = 0; i < 2; i++) {
        if (point[i] + length[!i] > MAX_INSTR)
            length[!i] = MAX_INSTR - point[i];
        if (point[i] + length[i] > parents[i].length[LEN_ABSOLUTE])
            length[i] = parents[i].length[LEN_ABSOLUTE] - point[i];
    }

    for(int j = 0; j < 2; j++) {
        for(int i = 0; i < point[j]; i++) {
            temp[j].instructions[i] = parents[j].instructions[i];
        }
        for(int i = 0; i < length[!j]; i++ ) {
            temp[j].instructions[i+point[j]] = parents[!j].instructions[i+point[!j]];
        }
        for(int i = 0; i < parents[j].length[LEN_ABSOLUTE] - point[j] - length[j]; i++) {
            temp[j].instructions[i+point[j]+length[!j]] = parents[j].instructions[i+point[j]+length[j]];
        }
        temp[j].length[LEN_ABSOLUTE] = point[j] + length[!j] + (parents[j].length[LEN_ABSOLUTE] - point[j] - length[j]);
        temp[j].fitness = INT_MAX;
    }
    memcpy(parents, temp, sizeof(*parents) *2);
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
    program_t eff_prog;
    int correct;
    int fitness[2];
    int idx[2];
    float probabilities[3] = { 0.4, 0.4, 0.2 };
    program_t winners[2];
//     int temperature = STARTTEMP;
//     int gototemp = STARTTEMP;
//     int iterations_since_change = 0;
//     int oldbestbcost = 1<<30;
//     int bestbcost = 1<<30;

    /* get the current calendar time */
    ltime = time(NULL);
    srand(ltime);

    fitness[0] = INT_MAX;
    fitness[1] = 0;

    init_levels();
    init_srcregisters();
    init_resultregisters();
    init_programs(programs);


    for(int i = 0; i < NUM_PROGRAMS; i++) {
        program_t *prog = &programs[i];
//         print_instructions(eff_prog.instructions, eff_prog.length, 1);
        effective_program(prog);
        printf("length (absolute effective)= %d %d, ", prog->length[LEN_ABSOLUTE], prog->length[LEN_EFFECTIVE]);
        run_program(prog, 0);
        result_fitness(prog);
        if (prog->fitness < fitness[0]) {
            fitness[0] = prog->fitness;
            idx[0] = i;
        }
        if (prog->fitness > fitness[1]) {
            fitness[1] = prog->fitness;
            idx[1] = i;
        }

        printf("fitness = %d\n", prog->fitness);
    }

    /* Best program replaces the worst, with a random chance at mutation */
    memcpy(&programs[idx[1]], &programs[idx[0]], sizeof(programs[0]));
    mutate_program(&programs[idx[1]], probabilities);
    effective_program(&programs[idx[1]]);
    run_program(&programs[idx[1]], 0);
    result_fitness(&programs[idx[1]]);
    printf("fitness = %d\n", programs[idx[1]].fitness);
    while (fitness[0] > 0) {
        run_tournament(programs, &winners[0], 8);
        run_tournament(programs, &winners[1], 8);
//         for(int i = 0; i < 2; i++)
//             printf("cost = %d\n", winners[i].cost);
        crossover(winners, 5, 50);
        for(int i = 0; i < 2; i++) {
            if (rand() < RAND_MAX * 0.75)
                mutate_program(&winners[i], probabilities);
            effective_program(&winners[i]);
            run_program(&winners[i], 0);
            result_fitness(&winners[i]);
//             printf("cost = %d\n", winners[i].cost);
        }
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < NUM_PROGRAMS; i++) {
                if(programs[i].fitness > winners[j].fitness) {
                    memcpy(&programs[i], &winners[j], sizeof(winners[j]));
                    break;
                }
            }
        }
        for(int i = 0; i < NUM_PROGRAMS; i++) {
            if (fitness[0] > programs[i].fitness) {
                fitness[0] = programs[i].fitness;
                effective_program(&programs[i]);
                run_program(&programs[i], 1);
                print_program(&programs[i], 0);
//                 printf("fitness = %d\n", programs[i].fitness);
                printf("\n");
            }
        }
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
