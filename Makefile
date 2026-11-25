CC = qcc
CFLAGS = -Vgcc_ntox86_64 -O2 -Wall -Wextra

brakewatch: src/brakewatch.c src/scenario.c src/scenario.h
	$(CC) $(CFLAGS) -o $@ src/brakewatch.c src/scenario.c

clean:
	rm -f brakewatch latency_*.csv
