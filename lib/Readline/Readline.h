#ifndef _READLINE_H
#define _READLINE_H

#include <Stream.h>

#define READLINE_MAX	128

class ReadLine {
public:
	ReadLine() : bpos(0),complete(false) {}
//	virtual ~ReadLine();
	
	char* feed(Stream *stream);
	char *feed(char c);
private:
	char buffer[READLINE_MAX];
	int bpos;
	bool complete;
};

#endif
