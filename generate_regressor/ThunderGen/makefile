CC = g++
LFLAGS =
CFLAGS = -g
MAIN = main
MOD1 = ThunderPanda
MOD2 = gen_regr_fun


$(MAIN): $(MAIN).o $(MOD1).o $(MOD2).o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o $(MOD1).o $(MOD2).o $(LFLAGS)

$(MAIN).o: $(MAIN).cpp $(MOD1).h $(MOD2).h
	$(CC) $(CFLAGS) -c $(MAIN).cpp $(LFLAGS)

$(MOD1).o: $(MOD1).cpp $(MOD1).h
	$(CC) $(CFLAGS) -c $(MOD1).cpp $(LFLAGS)

$(MOD2).o: $(MOD2).cpp $(MOD2).h
	$(CC) $(CFLAGS) -c $(MOD2).cpp $(LFLAGS)

clean:
	rm -f $(MAIN).o $(MAIN)
