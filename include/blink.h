#ifndef BLINK_H
#define BLINK_H

#include <stdint.h>

namespace Blink {



    class BLINK {
        public:
            BLINK(const uint32_t delaytime, const uint32_t brightness);

            uint32_t compute(const uint32_t delaytime);

            void setDelaytime(const uint32_t value);
            void setBrightness(const uint32_t value);
        private:
            int32_t brightness;
        protected:
            int32_t delaytime;

    };

}



#endif 