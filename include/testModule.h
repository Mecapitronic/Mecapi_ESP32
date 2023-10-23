#ifndef C0C4AD91_075C_4E09_A32E_1DE70EF23269
#define C0C4AD91_075C_4E09_A32E_1DE70EF23269

#include "ESP32_Helper.h"
using namespace Printer;

class testModule : public IModule
{
   public:
    testModule(void);
    // virtual ~testModule();
    void Initialisation();
    void SetConfig();
    bool ReadSerial();
    void SendSerial();
    void Analyse();
    // void CreateQueue(int size = 0);
};
#endif /* C0C4AD91_075C_4E09_A32E_1DE70EF23269 */
