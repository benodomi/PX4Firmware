#include <drivers/device/i2c.h>

#define I2C_DUMMY_BUS_DEFAULT PX4_I2C_BUS_EXPANSION
#define I2C_DUMMY_BASEADDR 0x51
#define I2C_DUMMY_DEVICE_PATH "/dev/i2c-dummy"


class I2C_DUMMY : public device::I2C
{
public:
	I2C_DUMMY( int bus = I2C_DUMMY_BUS_DEFAULT,
	      int address = I2C_DUMMY_BASEADDR);
	virtual ~I2C_DUMMY();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

   int getCounter();
   void resetCounter();

protected:
	virtual int			probe();

private:
   uint8_t readRegister(uint8_t reg);
   void setRegister(uint8_t reg, uint8_t value);

};

I2C_DUMMY::I2C_DUMMY( int bus, int address)
   :I2C("I2C-DUMMY", I2C_DUMMY_DEVICE_PATH, bus, address)
{}

I2C_DUMMY::~I2C_DUMMY()
{}

int 
I2C_DUMMY::init()
{
   int ret = PX4_ERROR;
   /* do I2C init (and probe) first */
	if (I2C::init() != OK) 
   {
		return ret;
   }

   set_device_address(I2C_DUMMY_BASEADDR);

   return PX4_OK;
}

int
I2C_DUMMY::probe()
{
   return PX4_OK;
}

int
I2C_DUMMY::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
   return I2C::ioctl(filp, cmd, arg);
}

ssize_t
I2C_DUMMY::read(device::file_t *filp, char *buffer, size_t buflen)
{
   return 0;
}

int
I2C_DUMMY::getCounter()
{
   uint8_t a = readRegister(0x01);
   uint8_t b = readRegister(0x02);
   uint8_t c = readRegister(0x03);

   return int((a&0x0f)*1 + ((a&0xf0)>>4)*10 + (b&0x0f)*100 + ((b&0xf0)>>4)*1000+ (c&0x0f)*10000 + ((c&0xf0)>>4)*1000000);
}

void
I2C_DUMMY::resetCounter()
{	
        setRegister(0x01,0x00);
        setRegister(0x02,0x00);
        setRegister(0x03,0x00);
}

void 
I2C_DUMMY::setRegister(uint8_t reg, uint8_t value)
{
      uint8_t buff[2];
      buff[0]=reg;
      buff[1]=value;
      int ret=transfer(buff, 2, nullptr, 0);

      if (OK != ret) {
		      PX4_DEBUG("I2C_DUMMY::setRegister : i2c::transfer returned %d", ret); 
	      }
}

uint8_t 
I2C_DUMMY::readRegister(uint8_t reg)
{
      uint8_t rcv;
      int ret=transfer(&reg, 1, nullptr, 0);

         if (OK != ret) {
		      PX4_DEBUG("I2C_DUMMY::readRegister : i2c::transfer returned %d", ret); 
	      }

      ret=transfer(nullptr, 0, &rcv, 1);

         if (OK != ret) {
		      PX4_DEBUG("I2C_DUMMY::readRegister : i2c::transfer returned %d", ret); 
	      }


      return rcv;
}

////////////////////////////////////MAIN////////////////////////////////

extern "C" __EXPORT int i2c_dummy_main(int argc, char *argv[]);

/*namespace i2c_dummy
{

   I2C_DUMMY *g_dev;

}//namespace i2c-dummy*/

I2C_DUMMY *g_dev;

int i2c_dummy_main(int argc, char *argv[])
{

   g_dev = new I2C_DUMMY();
   g_dev ->init();

   for(int i=0; i < 60; i++)
   {
      g_dev->resetCounter(); 
      usleep(1000000);
            
      PX4_INFO("Hello Sky: %d", g_dev->getCounter());
      //PX4_INFO("Reset.");
   }

   delete g_dev;
   g_dev = nullptr;

   return PX4_OK;
}



