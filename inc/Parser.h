/*
 * Parser.h
 *
 *  Created on: Oct 1, 2019
 *      Author: Shirmp (Quang Vu
 */

#ifndef PARSER_H_
#define PARSER_H_
#include <string>

/*This class receive the code from UART
 *also publish the control info to the controller*/
class Parser {
public:
	Parser() {}
	~Parser() {}
	std::string Parse(std::string input);
	enum command {
		opening,
		limitStatus,
		newPen,
		setPen,
		setDim,
		setLaser,
		moveOrg,
		move,
	};
private:
	std::string code;
};


#endif /* PARSER_H_ */
