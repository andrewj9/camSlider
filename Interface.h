// Interface.h

#ifndef _INTERFACE_h
#define _INTERFACE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

class Interface {
public:
	Interface();
	int getVal();
	void setVal(int val);
	void display();
	String toString();
private:
	String _title;
	
};


class Menu:public Interface {
public:
	Menu(String title);
	Interface getItems();
	void setItems(Interface item1);
	void setItems(Interface item1, Interface item2);
	void setItems(Interface item1, Interface item2, Interface item3);
	void setItems(Interface item1, Interface item2, Interface item3, Interface item4);
private:
	Interface _items[4];
};

class Option :public Interface {
public:
	Option(String title);
	int getVal();
	void setVal();
	int getRange(int limit);
	void setRange(int min, int max);
	void setMultiplier(int multiplier);
	void hasAlternate(bool doesIt);
	void setAlternate(String alt);
	void increment(int inc);
	int getPrevious();
	Interface getItems();

private:
	Interface _items[4];
	int _range[2];
	int _value;
	int _alternate[3];
	int _previousValue;
	bool _mostRecent;
	bool _zeroable;
	bool _rangeable;
	bool _displayAlternate;
	int _multiplier;
};

class Action :public Interface {
public:
	Action();

private:
	Interface _items[4];
};

class Camera :public Interface {
public:
	Camera();

private:
	Interface _items[4];
};

class Ready :public Interface {
public:
	Ready();

private:
	Interface _items[4];
};