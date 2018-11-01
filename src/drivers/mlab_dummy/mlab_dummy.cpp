#include <drivers/device/i2c.h>

#include <px4_workqueue.h>

#define MLAB_DUMMY_BUS_DEFAULT PX4_I2C_BUS_EXPANSION
#define MLAB_DUMMY_BASEADDR 0x51
#define MLAB_DUMMY_DEVICE_PATH "/dev/i2c-dummy"


class MLAB_DUMMY : public device::I2C
{
public:
	MLAB_DUMMY( int bus = MLAB_DUMMY_BUS_DEFAULT,
	      int address = MLAB_DUMMY_BASEADDR);
	virtual ~MLAB_DUMMY();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);


	virtual int			probe();

   void start();
   void stop();

   int getValue(); //get measured data


private:
   int getCounter();
   void resetCounter();

   uint8_t readRegister(uint8_t reg);
   void setRegister(uint8_t reg, uint8_t value);

   //workqueue
   work_s				_work{};
   static void			workqueue_callback(void *arg);
   void              cycle();
   int               interval_us;
   int               measuredData;

};

MLAB_DUMMY::MLAB_DUMMY( int bus, int address)
   :I2C("I2C-DUMMY", MLAB_DUMMY_DEVICE_PATH, bus, address)
{
   interval_us=1000000;
   measuredData=-1;
}

MLAB_DUMMY::~MLAB_DUMMY()
{}

int 
MLAB_DUMMY::init()
{
   int ret = PX4_ERROR;
   /* do I2C init (and probe) first */
	if (I2C::init() != OK) 
   {
		return ret;
   }

   set_device_address(MLAB_DUMMY_BASEADDR);
   
   //set counter mode
   setRegister(0x00,0b00100000);

   return PX4_OK;
}

int
MLAB_DUMMY::probe()
{
   return PX4_OK;
}

int
MLAB_DUMMY::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
   return I2C::ioctl(filp, cmd, arg);
}

ssize_t
MLAB_DUMMY::read(device::file_t *filp, char *buffer, size_t buflen)
{
   return 0;
}

int
MLAB_DUMMY::getCounter()
{
   uint8_t a = readRegister(0x01);
   uint8_t b = readRegister(0x02);
   uint8_t c = readRegister(0x03);

   return int((a&0x0f)*1 + ((a&0xf0)>>4)*10 + (b&0x0f)*100 + ((b&0xf0)>>4)*1000+ (c&0x0f)*10000 + ((c&0xf0)>>4)*1000000);
}

void
MLAB_DUMMY::resetCounter()
{	
        setRegister(0x01,0x00);
        setRegister(0x02,0x00);
        setRegister(0x03,0x00);
}

void 
MLAB_DUMMY::setRegister(uint8_t reg, uint8_t value)
{
      uint8_t buff[2];
      buff[0]=reg;
      buff[1]=value;
      int ret=transfer(buff, 2, nullptr, 0);

      if (OK != ret) {
		      PX4_DEBUG("MLAB_DUMMY::setRegister : i2c::transfer returned %d", ret); 
	      }
}

uint8_t 
MLAB_DUMMY::readRegister(uint8_t reg)
{
      uint8_t rcv;
      int ret=transfer(&reg, 1, &rcv, 1);

      if (OK != ret) {
	      PX4_DEBUG("MLAB_DUMMY::readRegister : i2c::transfer returned %d", ret); 
      }

      return rcv;
}

int 
MLAB_DUMMY::getValue()
{
      return measuredData;
}

//workqueue
void
MLAB_DUMMY::start()
{
   resetCounter();
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MLAB_DUMMY::workqueue_callback, this, USEC2TICK(interval_us));
}

void
MLAB_DUMMY::stop()
{
	work_cancel(HPWORK, &_work);
}

void 
MLAB_DUMMY::cycle()
{
   measuredData=getCounter();
   PX4_INFO("Measured Data: %d", measuredData);
   resetCounter();
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MLAB_DUMMY::workqueue_callback, this, USEC2TICK(interval_us));
}

void
MLAB_DUMMY::workqueue_callback(void *arg)
{
   MLAB_DUMMY *dev = (MLAB_DUMMY *)arg;
   dev->cycle();
}

////////////////////////////////////MAIN////////////////////////////////

extern "C" __EXPORT int mlab_dummy_main(int argc, char *argv[]);

namespace mlab_dummy
{

   MLAB_DUMMY *g_dev=nullptr;;
   void start();
   void stop();

   void start()
   {
         if(g_dev==nullptr)
         {
            g_dev = new MLAB_DUMMY();
            g_dev -> init();
            g_dev -> start();
         }
         else
         {
               PX4_INFO("MLAB_DUMMY driver already started");
         }
   }

   void stop()
   {
         g_dev -> stop();
         delete g_dev;
         g_dev = nullptr;
   }

}//namespace i2c-dummy*/

MLAB_DUMMY *g_dev;

int mlab_dummy_main(int argc, char *argv[])
{
   if(argc==2)
   {
      if(argv[1][0]=='s')
      {
         mlab_dummy::start();
         PX4_INFO("Starting MLAB_DUMMY driver");
      }


      if(argv[1][0]=='b')
      {
         mlab_dummy::stop();
         PX4_INFO("Stoping MLAB_DUMMY driver");
      }
      
   }
   else
   {
         PX4_INFO("mlab_dummy s|b");
   }
   return PX4_OK;
}



