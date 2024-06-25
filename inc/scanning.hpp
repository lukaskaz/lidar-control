#include "samples.hpp"
#include "serial.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

using Measurement = std::pair<bool, SampleData>;

class Normalscan
{
  public:
    explicit Normalscan(std::shared_ptr<serial> serialIf) : serialIf{serialIf}
    {}
    ~Normalscan()
    {
        stopscan();
    }

    void run();

  private:
    std::shared_ptr<serial> serialIf;
    Observer observer;

    void requestscan();
    Measurement getdata(bool);
    void stopscan();
};

class Expressscan
{
  public:
    explicit Expressscan(std::shared_ptr<serial> serialIf) : serialIf{serialIf}
    {}
    ~Expressscan()
    {
        stopscan();
    }

    void runlegacy();
    void rundense();

  private:
    std::shared_ptr<serial> serialIf;
    Observer observer;

    void requestscan();
    std::pair<double, std::vector<uint8_t>> getbasedata(bool);
    std::array<Measurement, 2> getlegacydata(std::vector<uint8_t>&&, double,
                                             double, uint8_t);
    Measurement getdensedata(std::vector<uint8_t>&&, double, double, uint8_t);
    void stopscan();
};
