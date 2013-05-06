#include <fstream>
#include <iostream>
#include <string>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "detector.h"

#include "standart_deviation.h"

#include <ctime>

using namespace cv;
using namespace std;

/**
 * Funkce fexists
 * 
 * Testuje zda soubor existuje
 * 
 * @param const char *filename          jmeno souboru
 * 
 * @return bool                         true pokud funkce existuje
 */
bool fexists(const char *filename)
{
  ifstream ifile(filename);
  return ifile.good();
}

/**
 * Funkce fexists
 * 
 * Testuje zda soubor existuje
 * 
 * @param string dirName                jméno adresáře
 * @param vector<string>& files         výstupní vektor souborů
 * 
 * @return bool                         true vše proběhlo v pořádku
 */
bool readDirectory(string dirName, vector<string>& files)
{
  files.clear();

  string filepath;
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  
  dp = opendir( dirName.c_str() );
  if (dp == NULL)
  {
    cout << "Error(" << errno << ") opening " << dirName << endl;
    return false;
  }
  
  while ((dirp = readdir( dp )))
  {
    filepath = dirName + "/" + dirp->d_name;
    
    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;
    
    // Endeavor to read a single number from the file and display it
    string ending = filepath.substr(filepath.find_last_of(".") + 1);
    if(ending == "png" || ending == "bmp" || ending == "jpg")
    {
      files.push_back(filepath);
    }    
  }
  
  sort(files.begin(), files.end());
  
  closedir( dp );
  
  return true;
}


void writeOutput(vector<vector<vector<Point2f> > > output, vector<string> files, string fileName)
{
  ofstream f;
  f.open(fileName);
  if (f.is_open())
  {
    for(int i = 0; i < (int)output.size(); i++)
    {
      f << "--> START file: " << files.at(i) << endl;
      for(int j = 0;j < (int)output.at(i).size(); j++)
      {
        for(int k = 0; k < (int)output.at(i).at(j).size(); k++)
        {
          f << "[" << output.at(i).at(j).at(k).x << ", " << output.at(i).at(j).at(k).y << "] ";
        }
        f << endl;
      }
      f << "--> END file: " << files.at(i) << endl << endl;
    }
  }
  f.close();
}

/**
 * funkce handleParameters
 *
 * Zpracuje parametry programu
 *
 * @param   int argc                  pocet argumentu
 * @param   char* argv[]              argumenty
 * @param   string &dirName           jmeno slozky se vstupnima obrazkama
 * @param   string &configFile        jméno konfiguračního souboru
 * @param   string &outputFile        jméno výstupního souboru
 * @param   bool &showImages          kontroluje zobrazování obrázků
 * @param   bool &writeImages         kontroluje zápis obrázků
 *
 * @return  bool                      uspech
 */
bool handleParameters(int argc, char* argv[], string &dirName, string &configFile, string &outputFile, bool &showImages, bool &writeImages, bool &debugOutput, int& lineWidth)
{
  showImages = writeImages = debugOutput = false;
  dirName = configFile = outputFile = "";
  lineWidth = 1;
  
  // zpracovani parametru prikazove radky
  for(int i = 1; i < argc; i++) 
  {
    if(string(argv[i]) == "-d" && i + 1 < argc) 
    {
      dirName = argv[++i];
    }
    else if(string(argv[i]) == "-c" && i + 1 < argc) 
    {
      configFile = argv[++i];
    }
    else if(string(argv[i]) == "-o" && i + 1 < argc) 
    {
      outputFile = argv[++i];
    }
    else if(string(argv[i]) == "-t" && i + 1 < argc) 
    {
      lineWidth = atoi(argv[++i]);
    }
    else if(string(argv[i]) == "-s") 
    {
      showImages = true;
    }
    else if(string(argv[i]) == "-w") 
    {
      writeImages = true;
    }
    else if(string(argv[i]) == "-a") 
    {
      debugOutput = true;
    }
    else if( string(argv[i]) == "-h") {
      cout << "Use: " << argv[0] << " [-s] [-w] [-a] [-t (1-9)] -c config_file -d directory" << endl;
      cout << "Detect UMF on cylinder." << endl;
      cout << "-s                       turn on show images" << endl;
      cout << "-w                       turn on write images" << endl;
      cout << "-t (1-9)                 drawing thickness" << endl;
      cout << "-a                       turn on show and write debug images" << endl;
      cout << "-c config_file           configuration file" << endl;
      cout << "-d directory             directory" << endl;
      cout << "-o output_file           file with result centers coordinates" << endl;
      return false;
    }
    else 
    {
      cerr << "Error: Unrecognized command line parameter \"" << argv[i] << "\" use -h to get more information." << endl;
      return false;
    }
  }

  // kontrola zadani parametru
  if(dirName.empty() || !fexists(configFile.c_str()) || outputFile.empty()) 
  {
    cerr << "Error: Some command line options were not specified. Use -h for more information." << endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{  
  vector<string> files;
  string dirName, ouputFile, configFile;
  bool showImages, writeImages, debugOutput;
  int lineWidth;
  
  if(!handleParameters(argc, argv, dirName, configFile, ouputFile, showImages, writeImages, debugOutput, lineWidth))
  {
    return 1;
  }
  
  if(!readDirectory(dirName, files))
  {
    return 1;
  }
  
  vector<vector<vector<Point2f> > > outputVector;
  vector<string> outputVectorName;
  
  CDetector* detector = new CDetector(configFile, lineWidth);
  
  //CStdDev* statistic = new CStdDev();
  
  for(int x = 0; x < (int)files.size(); x++) 
  {
    // clock_t begin = clock();
    // cout << "-------------- start file no. " << x << " [" << files.at(x) << "]--------------" << endl;
    // cout << "-------------- end file no. " << x << " [" << files.at(x) << "]--------------" << endl;
    vector<vector<Point2f> > tmpOutputVector;
    detector->runDetectorTest(files.at(x), showImages, writeImages, debugOutput, tmpOutputVector);
    
    outputVector.push_back(tmpOutputVector);
    outputVectorName.push_back(files.at(x));
    
    // clock_t end = clock();    
    // statistic->Push(double(end - begin) / CLOCKS_PER_SEC);
    
    if(showImages || debugOutput)
    {
      uchar c = (uchar)waitKey();
      
      if(c == 27)
      {
        break;
      }
    }    
  }
  
  writeOutput(outputVector, outputVectorName, ouputFile);
  
/*  
  cout << "mean: " << statistic->Mean() << endl;
  cout << "stdDev: " << statistic->StandardDeviation() << endl;
  cout << "max: " << statistic->Max() << endl;
  cout << "min: " << statistic->Min() << endl;
  
  delete statistic;
*/

  delete detector;
  
  return 0;
}
