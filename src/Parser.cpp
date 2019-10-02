#include "Parser.h"

std::string Parser::Parse(std::string input) {
	code = input;
	std::string temp = "";
	std::string output ="";
	auto it = input.cbegin() + 1;
	if (input[0] == 'M') {
		do {
			temp+=*it++;
		} while (*it != ' ' && it != input.cend());
		//filter for command after M
		switch (atoi(temp.c_str())) {
		case 1:
			output = "M1";
			break;
		case 2:
			output = "M2";
			break;
		case 4:
			output = "M4";
			break;
		case 5:
			output = "M5";
			break;
		case 10:
			output = "M10";
			break;
		case 11:
			output = "M11";
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
			output = "G1";
			break;
		case 28:
			output = "G28";
			break;
		default:
			output = "invalid";
			break;
		}
	} else {
		output = "invalid code";
	}
	return output;
}
