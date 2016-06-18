#include <cstdio>
#include <cstdint>
#include <complex>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace std;

struct frame_control_t
{
    uint16_t sequence_number: 4;
    uint16_t ack_request: 1;
    uint16_t low_power: 1;
    uint16_t reserved1: 2;
    uint16_t header_type: 4;
    uint16_t reserved2: 1;
    uint16_t beaming_info: 3;
    uint16_t reserver3: 4;
} __attribute__((packed));

struct packet {
    uint32_t home_id;
    uint8_t source_node_id;
    frame_control_t frame_control;
    uint8_t length;
    uint8_t dest_node_id;
    uint8_t command_class;
} __attribute__((packed));

uint8_t checksum( uint8_t* begin, uint8_t* end )
{
    uint8_t checksum( 0xff );
    for(uint8_t* it = begin; it != end; ++it )
    {
        checksum ^= *it;
    }
    return checksum;
}

void zwave_print(unsigned char* data, int len)
{
    chrono::milliseconds ms = chrono::duration_cast< chrono::milliseconds >(
        chrono::system_clock::now().time_since_epoch()
        );
    std::cerr << std::dec << std::setfill(' ') << std::setw(0);
    std::cerr << "[" << ms.count() << "] ";
    if ( len < sizeof(packet) || checksum( data, data + len - 1) != data[len-1] )
    {
        std::cerr << "Invalid packet!" << std::endl;
        return;
    }
    packet& p = *(packet*)data;
    std::cerr << std::hex << setfill('0') << std::setw(2);
    std::cerr << "HomeId: " << p.home_id;
    std::cerr << ", SourceNodeId: " << (int)p.source_node_id;
    std::cerr << std::hex << ", FC: " << *reinterpret_cast<uint16_t*>(&p.frame_control);
    std::cerr << std::dec;
    std::cerr << ", FC[ack_request=" << p.frame_control.ack_request;
    std::cerr << " low_power=" << p.frame_control.low_power;
    std::cerr << " header_type=" << p.frame_control.header_type;
    std::cerr << " beaming_info=" << p.frame_control.beaming_info;
    std::cerr << "], Length: " << std::dec << (int)p.length;
    std::cerr << ", DestNodeId: " << std::dec << (int)p.dest_node_id;
    std::cerr << ", CommandClass: " << std::dec << (int)p.command_class;
    std::cerr << ", Payload: ";
    std::cerr << std::hex << setfill('0');
    for (int i = sizeof(packet); i < len - 1; i++)
    {
        std::cerr << std::setw(2) << (int)data[i] << " ";
    }
    std::cerr << std::endl;
}

/*
   Algorithm

   The input signal is on the form s(t) = a*exp(-i*w*t+p)
   where a is the amplitude
   w if the angular frequncy, (in reality w is a function of t but we will ignore that)
   p if the phase difference

   We wish to find w...

   First we take the time derivative(s') of s
   s' = -i(w)*a*exp(-i*w*t+p)

   then we multiply s' by by conj(s) where conj is complex conjugation

   s'*conj(s) = -i(w)*a*exp(-i*w*t+p)*a*exp(i*w*t + p)
   = -i(w)*a*a

   finally we devide the result by the norm of s squared

   s'*conj(s) / |s|^2 = -i(w+p)

   Releated to the FSK demodulation, we know that w will fall out to two distinct values.
   w1 and w2, and that w2-w1 = dw.

   w will have the form w = wc +/- dw, where wc is the center frequnecy.

   wc + p will show up as a DC component in the s'*conj(s) / |s|^2 function.


   this function returns the angular frequency of the current QI sample.
   To get the actual frequency
   f = w* (sr / 2*pi)
   where sr is the sample rate
 */
static inline double fsk_demodulator(int re, int im)
{
  static complex<double> s1 = 0;
  static complex<double> s2 = 0;
  double w;
  complex<double> s((double) re, (double) im);

  double a2 = norm(s1);

  if (a2 > 0.0)
    {
      complex<double> ds = (s - s2) / 2.0; // the derivative
      complex<double> q = conj(s1) * ds;
      w = imag(q) / a2;
    }
  else
    {
      w = 0.0;
    }
  s2 = s1; // save 2 samp behind
  s1 = s;

  return w;
}




static inline double atan_fm_demodulator(int re, int im)
{
  static complex<double> s1 = 0;
  complex<double> s((double) re, (double) im);

  double d = arg(conj(s1) * s);
  s1 =s;
  return d;
}


#define NZEROS6 6
#define NPOLES6 6
#define GAIN6   4.570794845e+05


static inline double lp_filter1(double in)
{
  static float xv[NZEROS6+1], yv[NPOLES6+1];

  {
    xv[0] = xv[1];
    xv[1] = xv[2];
    xv[2] = xv[3];
    xv[3] = xv[4];
    xv[4] = xv[5];
    xv[5] = xv[6];
    xv[6] = in / GAIN6;
    yv[0] = yv[1];
    yv[1] = yv[2];
    yv[2] = yv[3];
    yv[3] = yv[4];
    yv[4] = yv[5];
    yv[5] = yv[6];
    yv[6] =   (xv[0] + xv[6]) + 6 * (xv[1] + xv[5]) + 15 * (xv[2] + xv[4])
              + 20 * xv[3]
              + ( -0.3862279890 * yv[0]) + (  2.6834487459 * yv[1])
              + ( -7.8013262392 * yv[2]) + ( 12.1514352550 * yv[3])
              + (-10.6996337410 * yv[4]) + (  5.0521639483 * yv[5]);
    return yv[6];
  }
}

static inline double lp_filter2(double in)
{
  static float xv[NZEROS6+1], yv[NPOLES6+1];

  {
    xv[0] = xv[1];
    xv[1] = xv[2];
    xv[2] = xv[3];
    xv[3] = xv[4];
    xv[4] = xv[5];
    xv[5] = xv[6];
    xv[6] = in / GAIN6;
    yv[0] = yv[1];
    yv[1] = yv[2];
    yv[2] = yv[3];
    yv[3] = yv[4];
    yv[4] = yv[5];
    yv[5] = yv[6];
    yv[6] =   (xv[0] + xv[6]) + 6 * (xv[1] + xv[5]) + 15 * (xv[2] + xv[4])
              + 20 * xv[3]
              + ( -0.3862279890 * yv[0]) + (  2.6834487459 * yv[1])
              + ( -7.8013262392 * yv[2]) + ( 12.1514352550 * yv[3])
              + (-10.6996337410 * yv[4]) + (  5.0521639483 * yv[5]);
    return yv[6];
  }
}


/**
 * Lowpass filter butterworth order 3 cutoff 100khz
 */
static inline double freq_filter(double in)
{

#define NZEROS 3
#define NPOLES 3
#define GAIN   3.681602264e+02

  static float xv[NZEROS + 1], yv[NPOLES + 1];

  xv[0] = xv[1];
  xv[1] = xv[2];
  xv[2] = xv[3];
  xv[3] = in / GAIN;
  yv[0] = yv[1];
  yv[1] = yv[2];
  yv[2] = yv[3];
  yv[3] = (xv[0] + xv[3]) + 3 * (xv[1] + xv[2]) + (0.5400688125 * yv[0])
          + (-1.9504598825 * yv[1]) + (2.3886614006 * yv[2]);
  return yv[3];

}

/*
 * Butterworth oder 3 low pass cutoff 10hz
 */
static inline double lock_filter(double in)
{
#define NZEROS1 3
#define NPOLES1 3
#define GAIN1   2.856028586e+05

  static float xv[NZEROS1 + 1], yv[NPOLES1 + 1];
  xv[0] = xv[1];
  xv[1] = xv[2];
  xv[2] = xv[3];
  xv[3] = in / GAIN1;
  yv[0] = yv[1];
  yv[1] = yv[2];
  yv[2] = yv[3];
  yv[3] = (xv[0] + xv[3]) + 3 * (xv[1] + xv[2]) + (0.9404830634 * yv[0])
          + (-2.8791542471 * yv[1]) + (2.9386431728 * yv[2]);
  return yv[3];

}

struct frame_state
{
  unsigned int bit_count;
  unsigned int data_len;
  unsigned char data[64];

  bool last_bit;

  int b_cnt;

  enum
  {
    B_PREAMP, B_SOF0, B_SOF1, B_DATA
  } state_b;
} fs;

enum
{
  S_IDLE, S_PREAMP, S_BITLOCK
} state = S_IDLE;

int pre_len = 0; //  # Length of preamble bit
int pre_cnt = 0;
double bit_len = 0;
double bit_cnt = 0.0;
double wc = 0; //  # center frequency
bool last_logic = false;
bool hasSignal = false;
bool msc; //Manchester
const int lead_in = 10;
double dr; //Datarate



/**
 * This program takes the output of rtl_sdr into stdin and decodes Z-Wave
 * frames. The sample rate is assumed to be 2.048 MHz
 */

/* 9.6k frame  of length  64 + preamble 10 */
#define SAMPLERATE 2048000
#define MAX_FRAME_DURATION (64+10)*8*(SAMPLERATE/9600)
int main(int argc, char** argv)
{
  double f, s, lock;
  size_t s_num = 0; //Sample number
  size_t f_num = 0; //Z-wave frame number
  complex<unsigned char> recorder[MAX_FRAME_DURATION]; //Cyclic buffer to store recorded frames
  int rec_ptr = 0;
  int frame_start;
  int enable_recorder, ch, fd;

  enable_recorder = 0;
  while ((ch = getopt(argc, argv, "r")) != -1)
    {
      switch (ch)
        {
        case 'r':
          enable_recorder = 1;
          break;
        case '?':
        default:
          printf("Usage: %s [-r] \n", argv[0]);
          printf("\t-r enable frame recorder\n");
          return -1;
        }
    }


  while (!feof(stdin))
    {
      unsigned char g[1024];
      fread(g, 1024, 1, stdin);

      for(int i=0; i < 1024; i+=2)
        {
          double re = (g[i] - 127);
          double im = (g[i+1] - 127);

          s_num++;

          re = lp_filter1(re);
          im = lp_filter2(im);

          //f = fsk_demodulator(re, im);
          f = atan_fm_demodulator(re,im);

          s = freq_filter(f);

          /*
           * We use a 12khz lowpass filter to lock on to a preable. When this value is "stable",
           * a preamble could be present, further more the value of lock, will correspond to the
           * center frequency of the fsk (wc)
           */
          lock = lock_filter(f);

          /*    If we are in bitlock mode, make sure that the signal does not derivate by more than
                1/2 seperation, TODO calculate 1/2 seperation
          */
          hasSignal = fabs(lock) > 0.01;

          if (hasSignal)
            {
              bool logic = (s - wc) < 0;

              if (state == S_IDLE)
                {
                  state = S_PREAMP;
                  pre_cnt = 0;
                  pre_len = 0;
                  frame_start = rec_ptr;
                  wc = lock;
                }
              else if (state == S_PREAMP)
                {
                  wc = 0.99*wc + lock*0.01;
                  pre_len++;
                  if (logic ^ last_logic)   //#edge trigger (rising and falling)
                    {
                      pre_cnt++;

                      if (pre_cnt == lead_in)    //# skip the first lead_in
                        {
                          pre_len = 0;
                        }
                      else if (pre_cnt > lead_in+20)     //Minimum preamble length is 10 bytes i.e 80 bits
                        {
                          state = S_BITLOCK;
                          fs.state_b = fs.B_PREAMP;
                          fs.last_bit = not logic;

                          bit_len = double(pre_len) / (pre_cnt - lead_in-1);
                          bit_cnt = 3 * bit_len / 4.0;
                          dr = SAMPLERATE/bit_len;
                          msc = dr < 15e3; //Should we use manchester encoding
                        }
                    }
                }
              else if (state == S_BITLOCK)
                {
                  if (logic ^ last_logic)
                    {
                      if(msc && (bit_cnt < bit_len/2.0))
                        {
                          bit_cnt = 1 * bit_len / 4.0; //#Re-sync on edges
                        }
                      else
                        {
                          bit_cnt = 3 * bit_len / 4.0; //#Re-sync on edges
                        }
                    }
                  else
                    {
                      bit_cnt = bit_cnt + 1.0;
                    }
                  if (bit_cnt >= bit_len)   // # new bit
                    {
                      //Sub state machine
                      if (fs.state_b == fs.B_PREAMP)
                        {
                          if (logic and fs.last_bit)
                            {
                              fs.state_b = fs.B_SOF1;
                              fs.b_cnt = 1; //This was the first SOF bit
                            }
                        }
                      else if (fs.state_b == fs.B_SOF0)
                        {
                          if (not logic)
                            {
                              if (fs.b_cnt == 4)
                                {
                                  fs.b_cnt = 0;
                                  fs.data_len = 0;
                                  fs.state_b = fs.B_DATA;
                                }
                            }
                          else
                            {
                              // SOF0 error (bit_len);
                              state = S_IDLE;
                            }
                        }
                      else if (fs.state_b == fs.B_SOF1)
                        {
                          if (logic)
                            {
                              if (fs.b_cnt == 4)
                                {
                                  fs.b_cnt = 0;
                                  fs.state_b = fs.B_SOF0;
                                }
                            }
                          else
                            {
                              // SOF1 error
                              state = S_IDLE;
                            }
                        }
                      else if (fs.state_b == fs.B_DATA)     //Payload bit
                        {
                          fs.data[fs.data_len] = (fs.data[fs.data_len] << 1)
                                                 | logic;
                          if ((fs.b_cnt & 7) == 0)
                            {
                              fs.data[++fs.data_len] = 0;
                            }
                        }
                      fs.last_bit = logic;
                      fs.b_cnt++;
                      bit_cnt = bit_cnt - bit_len;
                    }
                }
              last_logic = logic;
            }
          else     //# No LOCKs
            {
              if (state == S_BITLOCK && fs.state_b == fs.B_DATA)
                {
                  f_num++;
                  zwave_print(fs.data,fs.data_len);
                }
              fs.state_b = fs.B_PREAMP;
              state = S_IDLE;
            }
        }
    }
  return 0;
}
