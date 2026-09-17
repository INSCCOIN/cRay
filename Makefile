CC ?= gcc
CFLAGS ?= -O3 -ffast-math -Wall -Wextra -pthread
PREFIX ?= /usr/local

cRay: cRay.c ray.c ray.h
	$(CC) $(CFLAGS) -o cRay cRay.c ray.c -lm -pthread

install: cRay
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 cRay $(DESTDIR)$(PREFIX)/bin/cRay

clean:
	rm -f cRay
