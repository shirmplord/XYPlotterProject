#include "Parser.h"

void Parser::Parse(std::string input) {
	code = input;
	std::string temp = "";
	auto it = input.cbegin() + 1;
	if (input[0] == 'M') {
		do {
			temp+=*it++;
		} while (*it != ' ' && it != input.cend());
		output = 'M' + temp;
	} else if (input[0] == 'G') {
		do {
			temp+=*it++;
		} while (*it != ' ' && it != input.cend());
		output = 'G' + temp;
	}
	return;
}
