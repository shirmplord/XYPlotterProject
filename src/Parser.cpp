#include <string>
#include "Pen.cpp"

class Parser {
public:
	Parser();
	~Parser();
	void GetCode(std::string input);

private:
	std::string code;
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
};

void Parser::GetInput(std::string input) {
	code = input;
	return;
}
