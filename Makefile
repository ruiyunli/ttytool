all:
	gcc ttysend.c -o ttysend
	gcc ttyrecv.c -o ttyrecv

clean:
	rm -f ttysend ttyrecv
