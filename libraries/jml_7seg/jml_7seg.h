// the #include statment and code go here...

//depends on wire.h and Adafruit_GFX.h

class Jml_7seg {
	public:
		Jml_7seg(void);
		void init(void);
		void write(int, int = 10);
	private:
};