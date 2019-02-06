#include <io_lib/xml_parser.h>

namespace as64_
{

namespace io_
{

bool XmlParser::valid_key(const std::string& key)
{
    if(par_map.find(key) == par_map.end() )
    {
        std::cout << "Param_lib::XmlParser::valid_key: Parameter "+ key + " not found!" <<std::endl;
        return false;
    }
    return true;
}


std::complex<double> XmlParser::parse_cx(std::string str)
{
    double re,im;
    char i_ch;
    std::stringstream iss(str);

    // Parse full ...
    if(iss >> re >> im >> i_ch && (i_ch=='i'|| i_ch=='j')) return std::complex<double>(re,im);

    // ... or only imag
    iss.clear();
    iss.seekg(0,iss.beg);
    if(iss >> im >> i_ch && (i_ch=='i'|| i_ch=='j')) return std::complex<double>(0.0,im);

    // .. or only real
    iss.clear();
    iss.seekg(0,iss.beg);
    if(iss >> re) return std::complex<double>(re,0.0);

    // ... otherwise
    throw std::invalid_argument("Param_lib::XmlParser::parse_cx: Could not parse complex number!");
}

std::string errMsgAtLine(const std::string &fname, int line_number, const std::string &msg)
{
  std::ostringstream oss;

  oss << "[XmlParser Error]: Error parsing \"" << fname << "\", line " << line_number << ": " << msg;

  return oss.str();
}

std::string errMsg(const std::string &fname, const std::string &msg)
{
  std::ostringstream oss;

  oss << "[XmlParser Error]: Error parsing \"" << fname << ": " << msg;

  return oss.str();
}

XmlParser::XmlParser(const std::string& fname)
{
    std::ifstream fh;
    std::string line;
    size_t mark = 0;
    size_t end_mark = 0;
    int line_number = 0;

    // Clear parameter map
    par_map.clear();

    // Open file
    fh.open(fname.c_str());
    if (!fh)
    {
        throw std::ios_base::failure(errMsg(fname, "Could not find " + fname));
    }
    else
    {
        // Parse
        while (std::getline(fh, line))
        {
            line_number++;

            std::string keyS="";
            std::string dataS="";

            // Skip empty lines
            if (line.empty())
                continue;

            // Skip lines with only whitespace
            if(line.find_first_not_of("\t ")==std::string::npos)
                continue;

            // Remove comment
            mark = line.find("#");
            if(mark!=std::string::npos)
                line.erase(mark,line.length());

            // Do we have a '=' or a ':'
            mark = line.find("=");
            if(mark==std::string::npos) mark = line.find(":");
            if(mark!=std::string::npos)
            {
                // Find key
                keyS = line.substr(line.find_first_not_of("\t "),mark-line.find_first_not_of("\t "));
                keyS = keyS.substr(0,keyS.find_last_not_of("\t ")+1);

                // Find data
                dataS = line.substr(mark+1,line.length());
                dataS = dataS.substr(0,dataS.find_last_not_of("\t ")+1);

                // Do we have a vector/matrix
                mark = dataS.find("[");
                if(mark!=std::string::npos)
                {
                    dataS = dataS.substr(mark+1,dataS.length());
                    end_mark = dataS.find_last_of("]");
                    if (end_mark == std::string::npos) throw std::ios_base::failure(errMsgAtLine(fname, line_number,"Missing \"]\"."));
                    dataS = dataS.substr(0,dataS.find_last_of("]"));
                }

                // Do we have a string
                mark = dataS.find("\"");
                if(mark!=std::string::npos)
                {
                    dataS = dataS.substr(mark+1,dataS.length());
                    dataS = dataS.substr(0,dataS.find_last_of("\""));
                }
                else
                {
                  std::istringstream in_temp(dataS);
                  std::string temp;
                  in_temp >> temp;
                  if (!temp.compare("true")) dataS="1";
                  else if (!temp.compare("false")) dataS="0";
                  else throw std::ios_base::failure(errMsgAtLine(fname, line_number, "Unrecognized sequence: " + temp + ". If it's a string put it in quotes \"\""));
                }

                // Insert to map
                par_map.insert(std::pair<std::string, std::string>(keyS, dataS));
            }
        }

        // Close file
        fh.close();
    }
}


XmlParser::~XmlParser() {}


bool XmlParser::getString(const std::string key, std::string &value)
{
    if(!valid_key(key)) return false;

    value = par_map.find(key)->second;
    return true;
}

bool XmlParser::getVectorString(const std::string key, std::vector<std::string> &value)
{
    if(!valid_key(key)) return false;

    std::string col,str=par_map.find(key)->second;
    std::istringstream full_str(str);
    int K = static_cast<int>(std::count(str.begin(),str.end(),',')+1);
    value.resize(K);
    for(int k=0; k<K; k++)
    {
        std::getline(full_str, col, ',');
        std::stringstream iss(col);
        iss >> value[k];
    }
    return true;
}

bool XmlParser::getCxCol(const std::string key, arma::cx_vec &value)
{
    if(!valid_key(key)) return false;

    std::string row,str=par_map.find(key)->second;
    std::istringstream full_str(str);
    int K = static_cast<int>(std::count(str.begin(),str.end(),';')+1);
    arma::cx_vec x(K);
    for(int k=0; k<K; k++)
    {
        std::getline(full_str, row, ';');
        x(k) = parse_cx(row);
    }
    value = x;
    return true;
}


bool XmlParser::getCxRow(const std::string key, arma::cx_rowvec &value)
{
    if(!valid_key(key)) return false;

    std::string col,str=par_map.find(key)->second;
    std::istringstream full_str(str);
    int K = static_cast<int>(std::count(str.begin(),str.end(),',')+1);
    arma::cx_rowvec x(K);
    for(int k=0; k<K; k++)
    {
        std::getline(full_str, col, ',');
        x(k) = parse_cx(col);
    }
    value = x;
    return true;
}


bool XmlParser::getCxMat(const std::string key, arma::cx_mat &value)
{
    if(!valid_key(key)) return false;

    std::string full_str,row,col;
    std::istringstream iss_full;

    full_str = par_map.find(key)->second;
    int R = static_cast<int>(std::count(full_str.begin(),full_str.end(),';')+1);

    iss_full.str(full_str);
    std::getline(iss_full, row, ';');
    int C = static_cast<int>(std::count(row.begin(),row.end(),',')+1);

    arma::cx_mat x(R,C);

    iss_full.seekg(0,iss_full.beg);
    for(int r=0; r<R; r++)
    {
        std::getline(iss_full, row, ';');
        std::istringstream iss_row(row);
        for(int c=0; c<C; c++)
        {
            std::getline(iss_row, col, ',');
            x(r,c)=parse_cx(col);
        }
    }
    value = x;
    return true;
}

bool XmlParser::getParam(const std::string key, std::vector<bool> &value)
{
  arma::Row<unsigned> v;
  bool ret = getRow(key, v);
  value.resize(v.size());
  for (int i=0;i<v.size();i++) value[i] = (bool)(v(i));

  return ret;
}

} // namespace io_

} // namespace as64_
