#ifndef OUTPUT_H
#define OUTPUT_H

#include <stdint.h>
#define ServoCenter 1500

namespace Output {



    class OUTPUTFUNC {
        public:
            OUTPUTFUNC(const int32_t outputValueLA, const uint32_t brightness);

            uint32_t mixer(const uint32_t outputValueLA);

            void setDelaytime(const uint32_t value);
            void setBrightness(const uint32_t value);
        private:
            int32_t brightness;
        protected:
            int32_t delaytime;

    };

}



#endif 