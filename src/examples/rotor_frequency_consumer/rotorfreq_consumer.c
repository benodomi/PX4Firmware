#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <poll.h>
#include <uORB/topics/rotor_frequency.h>
#include <uORB/uORB.h>

__EXPORT int rotorfreq_consumer_main(int argc, char *argv[]);

int rotorfreq_consumer_main(int argc, char *argv[])
{

    PX4_INFO("Hello Sky!");

    int sensor_sub_fd = orb_subscribe(ORB_ID(rotor_frequency));

    px4_pollfd_struct_t fds[] = {
      { .fd = sensor_sub_fd,   .events = POLLIN },
    };

   while (true) 
   {
      
      int poll_ret = px4_poll(fds, 1, 2000);
   
      if(poll_ret<=0)
      {
          PX4_INFO("en error occured when pooling messages");
      }
      else
      {

         if (fds[0].revents & POLLIN) 
         {
           /* obtained data for the first file descriptor */
           struct rotor_frequency_s raw;
           /* copy sensors raw data into local buffer */
           orb_copy(ORB_ID(rotor_frequency), sensor_sub_fd, &raw);

           PX4_INFO("Rotor frequency:\t%f count: %d",
                       (double)raw.indicated_frequency_hz,
                        (int) raw.count);
          } 
      }
   }


    return OK;
}
