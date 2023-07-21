CC = g++
LFLAGS = -lcasadi -std=c++17
CFLAGS = -g
MAIN = main_genCode
MOD1 = SLregressor
MOD2 = RegBasic
MOD3 = FrameOffset
MOD4 = casadi

$(MAIN): $(MAIN).o $(MOD1).o $(MOD2).o $(MOD3).o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o $(MOD1).o $(MOD2).o $(MOD3).o $(LFLAGS)

$(MAIN).o: $(MAIN).cpp $(MOD1).h $(MOD2).h $(MOD3).h
	$(CC) $(CFLAGS) -c $(MAIN).cpp $(LFLAGS)

$(MOD1).o: $(MOD1).cpp $(MOD1).h
	$(CC) $(CFLAGS) -c $(MOD1).cpp $(LFLAGS)

$(MOD2).o: $(MOD2).cpp $(MOD2).h
	$(CC) $(CFLAGS) -c $(MOD2).cpp $(LFLAGS)

$(MOD3).o: $(MOD3).cpp $(MOD3).h
	$(CC) $(CFLAGS) -c $(MOD3).cpp $(LFLAGS)

clean:
	rm -f $(MAIN).o $(MAIN)
