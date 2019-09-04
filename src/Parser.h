#include <string>

/*This class receive the code from UART
 *also publish the control info to the controller*/
class Parser {
public:
	Parser() {}
	~Parser() {}
	void Parse(std::string input);
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
	std::string output;
private:
	std::string code;
};
