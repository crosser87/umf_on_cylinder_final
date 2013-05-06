#ifndef _UMF_PIXEL_COUNTER_H_
#define _UMF_PIXEL_COUNTER_H_

template <class T>
class Singleton
{
public:
	static T* Instance() {
		if(!m_pInstance) m_pInstance = new T;
		assert(m_pInstance !=NULL);
		return m_pInstance;
	}
protected:
	Singleton();
	~Singleton();
private:
	Singleton(Singleton const&);
	Singleton& operator=(Singleton const&);
	static T* m_pInstance;
};

template <class T> T* Singleton<T>::m_pInstance=NULL;

class PixelCounter
{
public:
	PixelCounter() {this->pixelCount = 0;};
	~PixelCounter() {};
	void reset() {this->pixelCount = 0;};
	int getPixels() {return this->pixelCount;};
	void add(int i = 1) {this->pixelCount+=i;};
private:
	int pixelCount;
};

typedef Singleton<PixelCounter> PCSingleton;


#endif
