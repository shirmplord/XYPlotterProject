#include "Parser.h"

void Parser::Parse(std::string input) {
	code = input;
	std::string temp = "";
	auto it = input.cbegin() + 1;
	if (input[0] == 'M') {
		do {
			temp+=*it++;
		} while (*it != ' ' && it != input.cend());
		//filter for command after M
		switch (atoi(temp.c_str())) {
		case 1:
		case 2:
		case 4:
		case 5:
		case 10:
		case 11:
			output = "M" + temp;
			break;
		default:
			output = "invalid";
			break;
		}
	} else if (input[0] == 'G') {
		do {
			temp+=*it++;
		} while (*it != ' ' && it != input.cend());
		//filter for command after G
		switch (atoi(temp.c_str())) {
		case 1:
		case 28:
			output = 'G' + temp;
			break;
		default:
			output = "invalid";
			break;
		}
	} else {
		output = "invalid code";
	}
	return;
}
