#ifndef __TOOLS__CPP__
#define __TOOLS__CPP__

#include "Tools.h"

#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <sys/times.h>


std::string 
Tools::itos(int i) 
{
   std::stringstream s;
   s << i;
   return s.str();
}


std::vector<string>
Tools::split(const std::string& s, const std::string& f) 
{
  std::vector<string> temp;
  if (f.empty()) {
    temp.push_back( s );
    return temp;
  }
  typedef std::string::const_iterator iter;
  const iter::difference_type f_size(distance(f.begin(), f.end()));
  iter i(s.begin());
  for (iter pos; (pos = std::search(i, s.end(), f.begin(), f.end()) ) != s.end(); ) {
    temp.push_back(string(i, pos));
    advance(pos, f_size);
    i = pos;
  }
  temp.push_back(string(i, s.end()));
  return temp;
}


std::vector<int> 
Tools::stringToIntVector(std::string text) 
{
   std::vector<int> tmp;

   bool stop = false;
   int pos;
   while (!stop) {
      pos = text.find_first_of(',');
      if (pos != -1) {
         tmp.push_back(atoi(text.substr(0, pos).c_str()));
         text = text.substr(pos + 1, text.length() - 1);
      } else {
         tmp.push_back(atoi(text.c_str()));
         stop = true;
      }
   }

   return tmp;
}




std::string 
Tools::indicesToString(std::string prefix, int i, int j) 
{
  int LABEL_INDEX_WIDTH = 2;
  std::ostringstream oss(prefix, std::ios_base::ate);
  oss << '[' << setw(LABEL_INDEX_WIDTH) << std::setfill('0') << i;
  if (j >= 0)
     oss << ',' << setw(LABEL_INDEX_WIDTH) << std::setfill('0') << j;
  oss << ']';
  return oss.str();
}


double 
Tools::CPUtime()
{
   tms t;
   times(&t);
   double ct = sysconf(_SC_CLK_TCK);
   return t.tms_utime/ct;
}


#endif // __TOOLS__CPP__
