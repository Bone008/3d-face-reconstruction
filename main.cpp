
#include <iostream>
#include <Eigen/Dense>

int main() {
	std::cout << "Hello Eigen!" << std::endl;

	Eigen::Matrix2f m;
	m << 1, 2, 3, 4;
	std::cout << m << std::endl;

	std::cin.get();
	return 0;
}
