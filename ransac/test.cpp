#include <iostream>
#include <vector>
#include <tuple>

int main(){
  std::vector<uint64_t> vec1;
  std::vector<uint64_t> vec2;

  for(int i = 0 ; i <5 ; i++){
    vec1.push_back(i);
    vec2.push_back(5+i);
  }

  std::tuple< std::vector<uint64_t> , std::vector<uint64_t> > tup;

  std::get<0>(tup) = vec1;
  std::get<2>(tup) = vec2;

  std::cout<<std::get<0>(tup)[1]<<std::endl;

  return 0;
}
