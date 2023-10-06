
template <class T>
class IFoo
{
   public:
    virtual void Initialisation() = 0;
    virtual void Config(T arg){
        // do something;
    };
};

template <class T>
class Foo : public IFoo<T>
{
   public:
    void functionA(){
        // do something;
    };
    void functionB(T arg){
        // do something;
    };
};
