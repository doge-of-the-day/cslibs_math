#include <fstream>
#include <cmath>

int main(int argc, char const *argv[])
{
	std::ofstream exponent_out("/tmp/exponent");
	std::ofstream fraction_out("/tmp/fraction");

	const auto write = [&](const double d) {
     	int exponent;
     	double fraction = std::frexp(d, &exponent);
     	exponent_out << (exponent - 1) << "\n";
     	fraction_out << (fraction - 0.5) << "\n";
	};

	const auto steps = 10000;
	const auto incr = 100.0 / static_cast<double>(steps);
	auto value = incr;

	for(auto i = 1 ; i < 10000 ; ++i, value+=incr)
	{
		write(value);
	}

	exponent_out.close();
	fraction_out.close();
	return 0;
}