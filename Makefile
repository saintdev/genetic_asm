all: default

SRCS = genetic_asm.c

OBJS = $(SRCS:%.c=%.o)
DEP  = depend

.PHONY: all default

default: $(DEP) genetic_asm

genetic_asm: $(OBJS)
	$(CC) -o $@ $+ $(LDFLAGS)

.depend:
	@$(RM) .depend
	@$(foreach SRC, $(SRCS) $(SRCCLI) $(SRCSO), $(CC) $(CFLAGS) $(SRC) -MT $(SRC:%.c=%.o) -MM -g0 1>> .depend;)

depend: .depend
ifneq ($(wildcard .depend),)
include .depend
endif

clean:
	$(RM) genetic_asm $(OBJ)
