# SMA Home Manager Simulator

Modified version of the [emeter simulator](https://github.com/RalfOGit/sma-emeter-simulator) to simulate a Sunny Home Manager 2.0 sending data to an EV charger.
For now it provides a linear ramp of PV feed in starting at 1500 Watt to 11kW and down again, incrementing by 10W every second.

```
git clone https://github.com/RalfOGit/libspeedwire.git
git clone git@github.com:danieltwagner/sma-emeter-simulator.git
cd sma-emeter-simulator
ln -s ../libspeedwire .
mkdir build && cd build
cmake ..
make
./sma-emeter-simulator 192.168.1.111
```