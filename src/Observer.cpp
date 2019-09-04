/*Observers are the objects that subscribe to a said topic*/
class Observer {
public:
	virtual void Update() = 0;
};

/*Subjects are the objects that publish to a said topic*/
class Subject {
public:
	void registerObserver(Observer* observer) = 0;
	void removeObserver(Observer* observer) = 0;
	void Notify() = 0;
};
