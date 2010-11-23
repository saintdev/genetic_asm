#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <limits.h>

#define NUM_REGS 16
#define MAX_INSTR 500
#define NUM_PROGRAMS 100
#define LEN_ABSOLUTE  0
#define LEN_EFFECTIVE 1
#define NUM_REF 3

typedef union reg {
    uint64_t q[2];
    uint32_t d[4];
    uint16_t wd[8];
    uint8_t  b[16];
} register_t;

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
    register_t registers[NUM_REGS];
    instruction_t instructions[MAX_INSTR];
    instruction_t effective[MAX_INSTR];
} program_t;

typedef struct reference {
    register_t input[NUM_REGS];
    register_t output[NUM_REGS];
    int num_regs_used[2];
} reference_t;

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

static uint16_t levels[8*8];
static uint16_t coeffs[8*8];
static const uint8_t allowedshuf[24] = { (0<<6)+(1<<4)+(2<<2)+(3<<0), (0<<6)+(1<<4)+(3<<2)+(2<<0),
                                         (0<<6)+(2<<4)+(3<<2)+(1<<0), (0<<6)+(2<<4)+(1<<2)+(3<<0),
                                         (0<<6)+(3<<4)+(2<<2)+(1<<0), (0<<6)+(3<<4)+(1<<2)+(2<<0),

                                         (1<<6)+(0<<4)+(2<<2)+(3<<0), (1<<6)+(0<<4)+(3<<2)+(2<<0),
                                         (1<<6)+(2<<4)+(3<<2)+(0<<0), (1<<6)+(2<<4)+(0<<2)+(3<<0),
                                         (1<<6)+(3<<4)+(2<<2)+(0<<0), (1<<6)+(3<<4)+(0<<2)+(2<<0),

                                         (2<<6)+(1<<4)+(0<<2)+(3<<0), (2<<6)+(1<<4)+(3<<2)+(0<<0),
                                         (2<<6)+(0<<4)+(3<<2)+(1<<0), (2<<6)+(0<<4)+(1<<2)+(3<<0),
                                         (2<<6)+(3<<4)+(0<<2)+(1<<0), (2<<6)+(3<<4)+(1<<2)+(0<<0),

                                         (3<<6)+(1<<4)+(2<<2)+(0<<0), (3<<6)+(1<<4)+(0<<2)+(2<<0),
                                         (3<<6)+(2<<4)+(0<<2)+(1<<0), (3<<6)+(2<<4)+(1<<2)+(0<<0),
                                         (3<<6)+(0<<4)+(2<<2)+(1<<0), (3<<6)+(0<<4)+(1<<2)+(2<<0) };

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

void init_levels_4x4(reference_t *ref)
{
    uint16_t coeffs[4*4];
    uint16_t levels[4*4];
    int r;

    ref->num_regs_used[0] = ref->num_regs_used[1] = 2;

    for(r = 0; r < ref->num_regs_used[0]; r++)
        memcpy(&coeffs[r*8], &ref->input[r], sizeof(coeffs[0]) * 8);
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,3,1) ZIG(11,2,2)
    ZIG(12,1,3) ZIG(13,2,3) ZIG(14,3,2) ZIG(15,3,3)
    for(int r = 0; r < ref->num_regs_used[1]; r++)
        memcpy(&ref->output[r], &levels[r*8], sizeof(ref->output[0]));
    for( ; r < NUM_REGS; r++)
        memset(&ref->output[r], 0, sizeof(ref->output[0]));
}

void print_instruction( instruction_t *instr, int debug )
{
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
        if (instr->operands[1] < NUM_REGS)
                                        printf("m%d", instr->operands[1]);
        else
                                        printf("0x%x", allowedshuf[instr->operands[1] - NUM_REGS]);
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

void print_register( register_t *reg, int type )
{
    switch (type) {
        case 0:
            for(int i = 0; i < 8; i++)
                printf("%u ", reg->wd[i]);
            break;
    }
}

void execute_instruction( instruction_t instr, register_t *registers )
{
    register_t temp;
    register_t *output = &registers[instr.operands[0]];
    register_t *input1 = &registers[instr.operands[1]];
    uint8_t imm = instr.operands[2];
    int i;

    switch( instr.opcode )
    {
        case PUNPCKLWD:
            temp.wd[0] = output->wd[0];
            temp.wd[2] = output->wd[1];
            temp.wd[4] = output->wd[2];
            temp.wd[6] = output->wd[3];
            temp.wd[1] = input1->wd[0];
            temp.wd[3] = input1->wd[1];
            temp.wd[5] = input1->wd[2];
            temp.wd[7] = input1->wd[3];
            break;
        case PUNPCKHWD:
            temp.wd[0] = output->wd[4];
            temp.wd[2] = output->wd[5];
            temp.wd[4] = output->wd[6];
            temp.wd[6] = output->wd[7];
            temp.wd[1] = input1->wd[4];
            temp.wd[3] = input1->wd[5];
            temp.wd[5] = input1->wd[6];
            temp.wd[7] = input1->wd[7];
            break;
        case PUNCPKLDQ:
            temp.d[0] = output->d[0];
            temp.d[2] = output->d[1];
            temp.d[1] = input1->d[0];
            temp.d[3] = input1->d[1];
            break;
        case PUNPCKHDQ:
            temp.d[0] = output->d[2];
            temp.d[2] = output->d[3];
            temp.d[1] = input1->d[2];
            temp.d[3] = input1->d[3];
            break;
        case PUNPCKLQDQ:
            temp.q[0] = output->q[0];
            temp.q[1] = input1->q[0];
            break;
        case PUNPCKHQDQ:
            temp.q[0] = output->q[1];
            temp.q[1] = input1->q[1];
            break;
        case MOVDQA:
            temp.q[0] = input1->q[0];
            temp.q[1] = input1->q[0];
            break;
        case PSLLDQ:
            if (imm > 16) imm = 16;
            for( i = 0; i < 16 - imm; i++ )
                temp.b[i] = output->b[i+imm];
            for( ; i < 16; i++ )
                temp.b[i] = 0;
            break;
        case PSRLDQ:
            if (imm > 16) imm = 16;
            for( i = 0; i < imm; i++ )
                temp.b[i] = 0;
            for( ; i < 16; i++ )
                temp.b[i] = output->b[i-imm];
            break;
        case PSLLQ:
            if (imm > 64) imm = 64;
            for (i = 0; i < 2; i++)
                temp.q[i] = output->q[i] << imm;
            break;
        case PSRLQ:
            if (imm > 64) imm = 64;
            for (i = 0; i < 2; i++)
                temp.q[i] = output->q[i] >> imm;
            break;
        case PSLLD:
            if (imm > 32) imm = 32;
            for (i = 0; i < 4; i++)
                temp.d[i] = output->d[i] << imm;
            break;
        case PSRLD:
            if (imm > 32) imm = 32;
            for (i = 0; i < 4; i++)
                temp.d[i] = output->d[i] >> imm;
            break;
        case PSHUFLW:
            temp.wd[0] = input1->wd[imm&0x3]; imm >>= 2;
            temp.wd[1] = input1->wd[imm&0x3]; imm >>= 2;
            temp.wd[2] = input1->wd[imm&0x3]; imm >>= 2;
            temp.wd[3] = input1->wd[imm&0x3];
            temp.q[1] = input1->q[1];
            break;
        case PSHUFHW:
            temp.q[0] = input1->q[0];
            temp.wd[4] = input1->wd[4+(imm&0x3)]; imm >>= 2;
            temp.wd[5] = input1->wd[4+(imm&0x3)]; imm >>= 2;
            temp.wd[6] = input1->wd[4+(imm&0x3)]; imm >>= 2;
            temp.wd[7] = input1->wd[4+(imm&0x3)];
            break;
        default:
            fprintf( stderr, "Error: unsupported instruction %d %d %d %d!\n", instr.opcode, instr.operands[0], instr.operands[1], instr.operands[2]);
            assert(instr.opcode < NUM_INSTR);
    }

    memcpy( output, &temp, sizeof(*output) );
}

void init_resultregisters(register_t *reference)
{
    int r, i;
    for(r=0;r<8;r++)
        for(i=0;i<8;i++)
            reference[r].wd[i] = levels[i+r*8];
}

void init_srcregisters(register_t *regs)
{
    for(int r = 0; r < NUM_REGS; r++)
        for(int i = 0; i < 4; i++)
            regs[r].d[i] = rand();
}

void init_registers(program_t *program, reference_t *ref)
{
    memcpy( program->registers, ref->input, sizeof(program->registers) );
}

void init_reference(reference_t *ref)
{
    init_levels_4x4(ref);
    for(int r = ref->num_regs_used[0]; r < NUM_REGS; r++)
        memset(&ref->input[r], 0, sizeof(ref->input[0]));
}

void init_programs(program_t *programs)
{
    for(int i = 0; i < NUM_PROGRAMS; i++) {
        program_t *program = &programs[i];
        program->length[LEN_ABSOLUTE] = (rand() % (96)) + 5;
        for(int j = 0; j < program->length[LEN_ABSOLUTE]; j++) {
            int instr = rand() % NUM_INSTR;
            int output = rand() % NUM_REGS;
            int input1 = NUM_REGS, input2 = 0;
            assert(j < MAX_INSTR);
            instruction_t *instruction = &program->instructions[j];

            /* FIXME: This should be completely random, instead of the guided randomnes we have below.
             * This may help generate more valid code, however.
             */
            input1 = rand() % NUM_REGS;
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

            assert(instr < NUM_INSTR);
            instruction->opcode = instr;
            instruction->operands[0] = output;
            instruction->operands[1] = input1;
            instruction->operands[2] = input2;
        }
    }
}

void effective_program(program_t *prog, int num_output_regs)
{
    uint8_t reg_eff[NUM_REGS] = {0};
    int i = prog->length[LEN_ABSOLUTE] - 1;
    int j;

    /* We want our results starting at r0. */
    for(int r = 0; r < num_output_regs; r++)
        reg_eff[r] = 1;

    while(i >= 0 && (prog->instructions[i].operands[0] != 0 && prog->instructions[i].operands[0] != 1)) {
        assert(i < MAX_INSTR && i >= 0);
        prog->instructions[i].flags = 0;
        i--;
    }
    for( ; i >= 0; i--) {
        instruction_t *instr = &prog->instructions[i];
        assert(i >= 0);

        instr->flags = 0;
        for (j = 0; j < NUM_REGS; j++)
            if (reg_eff[j] && instr->operands[0] == j)
                instr->flags = 1;

        if (!instr->flags)
            continue;
        if (instr->opcode >= PSHUFLW || instr->opcode == MOVDQA)
            reg_eff[instr->operands[0]] = 0;
        if (instr->operands[1] < NUM_REGS &&
            (instr->opcode < PSLLDQ || instr->opcode > PSRLD))
            reg_eff[instr->operands[1]] = 1;
    }

    for(j = i = 0; i < prog->length[LEN_ABSOLUTE]; i++) {
        assert(i < MAX_INSTR);
        if (!prog->instructions[i].flags)
            continue;
        prog->instructions[i].flags = 0;
        prog->effective[j++] = prog->instructions[i];
    }
    prog->length[LEN_EFFECTIVE] = j;
}

int run_program( program_t *program, reference_t *ref, int debug )
{
    init_registers(program, ref);
    if( debug ) {
//         printf("sourceregs: \n");
//         for(r=0; r<8; r++) {
//             for(j=0; j<8; j++)
//                 printf("%d ", srcregisters[r][j]);
//             printf("\n");
//         }
        printf("\ntargetregs:\n");
        for(int r = 0; r < ref->num_regs_used[0]; r++) {
            print_register(&ref->output[r], 0);
            printf("\n");
        }
    }
    for(int i = 0; i < program->length[LEN_EFFECTIVE]; i++ ) {
/*
        if(debug) {
            printf("regs: \n");
            for(r=0;r<NUM_REGS;r++) {
                for(j=0;j<8;j++)
                    printf("%d ",registers[r][j]);
                printf("\n");
            }
        }
*/
        execute_instruction( program->effective[i], program->registers );
    }
    if(debug) {
        printf("\nresultregs:\n");
        for(int r = 0; r < NUM_REGS; r++) {
            print_register(&program->registers[r], 0);
            printf("\n");
        }
    }
    return 1;
}

//#define CHECK_LOC if( i >= 2 && i <= 5 ) continue;
#define CHECK_LOC if( 0 ) continue;

int result_fitness( program_t *prog, reference_t *ref )
{
    int sumerror = 0;

    for( int r = 0; r < ref->num_regs_used[1]; r++ ) {
        int regerror = 0;
        for(int i = 0; i < 8; i++ )
            regerror += prog->registers[r].wd[i] != ref->output[r].wd[i];
        sumerror += regerror;
    }

    return sumerror*sumerror;
}

void result_cost( program_t *prog )
{
    /* TODO: Use instruction latency/thouroghput */
//     prog->cost = prog->length[LEN_EFFECTIVE];
//     if(!prog->cost)
        prog->cost = INT_MAX;
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

void mutate_program( program_t *prog, float probabilities[3] )
{
    if (prog->length[LEN_ABSOLUTE] == 0)
        return;
    int p = rand();
    int ins_idx = rand() % prog->length[LEN_ABSOLUTE];
    instruction_t *instr = &prog->instructions[ins_idx];

    assert(ins_idx < MAX_INSTR);

    if(p < RAND_MAX * probabilities[0])                                 /* Modify an instruction */
        instr->opcode = rand() % NUM_INSTR;
    else if (p < RAND_MAX * (probabilities[0] + probabilities[1])) {    /* Modify a regester */
        if (rand() < RAND_MAX / 2)
            instr->operands[0] = rand() % NUM_REGS;
        else
            instr->operands[1] = rand() % NUM_REGS;
    } else                                                              /* Modify a constant */
        instr->operands[2] = rand() % UINT8_MAX;

    assert(instr->opcode < NUM_INSTR);
    /* Invalidate existing fitness */
    prog->fitness = INT_MAX;
    prog->cost = 0;
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
    int idx = rand() % NUM_PROGRAMS;
    program_t *best = &programs[idx];

    for(int i = 0, j = 0; i < NUM_PROGRAMS - 1; i++) {
        if(i == idx)
            continue;
        contestants[j++] = i;
    }
    for(int i = 0; i < (size - 1); i++) {
        int ii = rand() % ((NUM_PROGRAMS-1)- (i+1));
        idx = contestants[ii];
        if(programs[idx].fitness < best->fitness)
            best = &programs[idx];
        else if(programs[idx].fitness == best->fitness)
            if(programs[idx].cost < best->cost)
                best = &programs[idx];
        for(int j = idx; j < NUM_PROGRAMS-1 - (i+1); j++)
            contestants[j] = contestants[j+1];
    }
    memcpy(winner, best, sizeof(*winner));
}

void crossover( program_t *parents, int delta_length, int delta_pos )
{
    program_t temp[2];
    /* FIXME: Respect delta_pos, delta_length */
    int point[2];
    int length[2];
    int j;

    for(int i = 0; i < 2; i++) {
        if (parents[i].length[LEN_ABSOLUTE] == 0)
            return;
        point[i] = rand() % parents[i].length[LEN_ABSOLUTE];
        length[i] = rand() % (parents[i].length[LEN_ABSOLUTE]+1 - point[i]);
    }

    for(int i = 0; i < 2; i++)
        if (parents[!i].length[LEN_ABSOLUTE] - length[!i] + length[i] > MAX_INSTR)
            length[i] = MAX_INSTR - (parents[!i].length[LEN_ABSOLUTE] - length[!i]);

    for(int i = 0; i < 2; i++) {
        for(j = 0; j < parents[i].length[LEN_ABSOLUTE] - length[i] + length[!i]; j++) {
            assert(j - length[!i] + length[i] < parents[i].length[LEN_ABSOLUTE]);
            if (j < point[i])
                temp[i].instructions[j] = parents[i].instructions[j];
            else if (j < point[i] + length[!i])
                temp[i].instructions[j] = parents[!i].instructions[j + point[!i] - point[i]];
            else
                temp[i].instructions[j] = parents[i].instructions[j - length[!i] + length[i]];
        }
        temp[i].length[LEN_ABSOLUTE] = j;
    }

    memcpy(parents, temp, sizeof(*parents) *2);
}

void analyse_program(program_t *prog, reference_t refs[NUM_REF])
{
    prog->fitness = 0;
    effective_program(prog, refs[0].num_regs_used[1]);
    for(int i = 0; i < NUM_REF; i++) {
        run_program(prog, &refs[i], 0);
        prog->fitness += result_fitness(prog, &refs[i]);
    }
    result_cost(prog);
}

int main()
{
    program_t programs[NUM_PROGRAMS];
    int ltime;
    int fitness[2];
    int cost[2];
    int idx[2] = { 0 };
    float probabilities[3] = { 0.4, 0.4, 0.2 };
    program_t winners[2];
    reference_t ref[NUM_REF];

    /* get the current calendar time */
    ltime = time(NULL);
    srand(ltime);

    fitness[0] = INT_MAX;
    fitness[1] = 0;
    cost[0] = INT_MAX;
    cost[1] = 0;

    for(int r = 0; r < 2; r++)
        for(int i = 0; i < 8; i++)
            ref[0].input[r].wd[i] = i+1+(r*8);
    init_reference(&ref[0]);

    for(int i = 1; i < NUM_REF; i++) {
        init_srcregisters(ref[i].input);
        init_reference(&ref[i]);
    }
    init_programs(programs);


    for(int i = 0; i < NUM_PROGRAMS; i++) {
        program_t *prog = &programs[i];
        analyse_program(prog, ref);
        printf("length (absolute effective)= %d %d, ", prog->length[LEN_ABSOLUTE], prog->length[LEN_EFFECTIVE]);

        if (prog->fitness < fitness[0]) {
            fitness[0] = prog->fitness;
            idx[0] = i;
        } else if (prog->fitness == fitness[0]) {
            if (prog->cost && prog->cost < cost[0]) {
                cost[0] = prog->cost;
                idx[0] = i;
            }
        }
        if (prog->fitness > fitness[1]) {
            fitness[1] = prog->fitness;
            idx[1] = i;
        } else if (prog->fitness == fitness[1]) {
            if (prog->cost > cost[1]) {
                cost[1] = prog->cost;
                idx[1] = i;
            }
        }

        printf("fitness = %d\n", prog->fitness);
    }

    /* Best program replaces the worst, with a random chance at mutation */
    memcpy(&programs[idx[1]], &programs[idx[0]], sizeof(programs[0]));
    mutate_program(&programs[idx[1]], probabilities);
    analyse_program(&programs[idx[1]], ref);
    printf("fitness = %d\n", programs[idx[1]].fitness);
    while (fitness[0] > 0) {
        run_tournament(programs, &winners[0], 8);
        run_tournament(programs, &winners[1], 8);
        crossover(winners, 5, 50);
        for(int i = 0; i < 2; i++) {
            if (rand() < RAND_MAX * 0.75)
                mutate_program(&winners[i], probabilities);
            analyse_program(&winners[i], ref);
        }
        for (int j = 0; j < 2; j++) {
            for (int i = 0; i < NUM_PROGRAMS; i++) {
                if(programs[i].fitness > winners[j].fitness) {
                    memcpy(&programs[i], &winners[j], sizeof(programs[0]));
                    break;
                }
            }
        }
        for(int i = 0; i < NUM_PROGRAMS; i++) {
            if (fitness[0] > programs[i].fitness) {
                fitness[0] = programs[i].fitness;
                effective_program(&programs[i], ref[0].num_regs_used[1]);
                run_program(&programs[i], &ref[0], 1);
                print_program(&programs[i], 0);
                printf("\n");
            }
        }
    }

    return 0;
}
