#include <vector>
#include <iostream>

int main(){
  std::vector<std::vector<double>> a;
  for(int i = 0; i < 10; i++){
    a.push_back(std::vector<double>());
  }
  a[2].push_back(2.0);
  std::cout << a[0].size() << std::endl;
  std::cout << a[2].size() << std::endl;
  return 0;
}
