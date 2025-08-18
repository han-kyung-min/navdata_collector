
#include <shared_mutex>  
#include <iostream>
#include <fstream>
#include <shared_mutex>                    // needed before Karto.h on gcc7/Ubuntu18

#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/nvp.hpp>

#include <slam_toolbox/serialization.hpp>  // declares serialization::read(...)
#include <karto_sdk/Mapper.h>
#include <karto_sdk/Karto.h>

int main(int argc, char** argv)
{
  if (argc < 3) {
	std::cerr << "usage: dump_posegraph <path/to/map.posegraph> <out.txt>\n";
	return 1;
  }
  const std::string pg_path = argv[1];
  const std::string out_txt = argv[2];

  karto::Mapper mapper;
  karto::Dataset dataset;
 
//   Most builds expose it here:
  std::cout << "pg_path: " << pg_path.c_str() << std::endl;
  bool ok = ::serialization::read(pg_path, mapper, dataset);
  
//  If you get “no matching function” for the above line, try the pointer form:
//  bool ok = ::serialization::read(pg_path, &mapper, &dataset);

  if (!ok) {
    std::cerr << "Failed to read posegraph: " << pg_path << "\n";
    return 2;
  }

  std::ofstream ofs(out_txt);
  if (!ofs.is_open()) {
    std::cerr << "Could not open output file: " << out_txt << "\n";
    return 3;
  }

//  ofs << "Total scans: " << scans.size() << "\n";
//  ofs << "id    x          y          theta\n";
//  ofs << "------------------------------------\n";
  const karto::LocalizedRangeScanVector scans = mapper.GetAllProcessedScans();
  std::cout << "Total scans: " << scans.size() << std::endl;
  ofs << std::fixed << std::setprecision(4);

  for (const auto* scan : scans)
  {
    const karto::Pose2& p = scan->GetCorrectedPose();
    const int id = scan->GetUniqueId();
    //*os << id << "," << p.GetX() << "," << p.GetY() << "," << p.GetHeading() << "\n";
    ofs << std::setw(6)  << id
        << std::setw(12) << p.GetX()
        << std::setw(12) << p.GetY()
        << std::setw(12) << p.GetHeading() << "\n";
  }
  ofs.close() ;
  
  printf("Deserialization completed \n");
  std::_Exit(0);

  return 0;
}
