all: filter_test

filter_test: filter_test.o filters.o
	$(CC) -o $@ $^

filter_test.o: filter_test.c
	$(CC) -c $< -o $@

filters.o: ../Core/Src/filters.c
	$(CC) -c $< -o $@

clean:
	rm -f filter_test filter_test.o filters.o
