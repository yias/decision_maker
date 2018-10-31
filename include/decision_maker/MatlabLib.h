

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>	
#include <vector>
//#include <conio.h>

#include "mat.h"
#include "mex.h"

#define BUFSIZE 256




struct eObjects{
	std::vector< std::vector<double> > reach[20];
};

struct sigNals{
	eObjects *ob;
	std::vector<double> mvc;
};



int writeMatFile(std::string filename, std::string valName, std::vector<double> cont){

    std::cout<<"\ntrying to save to "<<filename<<"\n";

	MATFile *pmat;
	mxArray *pa1;
	filename += ".mat";
	const char *file = filename.c_str();
//	char str[BUFSIZE];
	int status;

	double *data = new double[(int)cont.size()];
	for (int i = 0; i < (int)cont.size(); i++){
		data[i] = cont[i];
	}
	

	pmat = matOpen(file, "w");
	if (pmat == NULL) {
		printf("Error creating file %s\n", file);
		printf("(Do you have write permission in this directory?)\n");
		return(EXIT_FAILURE);
	}

	pa1 = mxCreateDoubleMatrix(1, (int)cont.size(), mxREAL);
	if (pa1 == NULL) {
		printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
		printf("Unable to create mxArray.\n");
		return(EXIT_FAILURE);
	}
	std::memcpy((void *)(mxGetPr(pa1)), (void *)data, cont.size()*sizeof(double));
	double *realPtr = mxGetPr(pa1);
	
	
    status = matPutVariable(pmat, valName.c_str(), pa1);
	if (status != 0) {
		printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
		return(EXIT_FAILURE);
	}



	mxDestroyArray(pa1);
	if (matClose(pmat) != 0) {
		printf("Error closing file %s\n", file);
		return(EXIT_FAILURE);
	}

	std::cout << "the data are saved to the file: " << filename << "\n";
	return(EXIT_SUCCESS);
}




int writeMatFile(std::string filename, std::string valName, std::vector<int> cont){

    MATFile *pmat;
    mxArray *pa1;
    filename += ".mat";
    const char *file = filename.c_str();
//	char str[BUFSIZE];
    int status;

    int *data = new int[(int)cont.size()];
    for (int i = 0; i < (int)cont.size(); i++){
        data[i] = cont[i];
    }


    pmat = matOpen(file, "w");
    if (pmat == NULL) {
        printf("Error creating file %s\n", file);
        printf("(Do you have write permission in this directory?)\n");
        return(EXIT_FAILURE);
    }

    //pa1 = mxCreateDoubleMatrix(1, (int)cont.size(), mxREAL);
    pa1 = mxCreateNumericMatrix(1, (int)cont.size(), mxINT16_CLASS, mxREAL);
    if (pa1 == NULL) {
        printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
        printf("Unable to create mxArray.\n");
        return(EXIT_FAILURE);
    }
    std::memcpy((void *)(mxGetPr(pa1)), (void *)data, cont.size()*sizeof(int));
    double *realPtr = mxGetPr(pa1);


    status = matPutVariable(pmat, valName.c_str(), pa1);
    if (status != 0) {
        printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
        return(EXIT_FAILURE);
    }



    mxDestroyArray(pa1);
    if (matClose(pmat) != 0) {
        printf("Error closing file %s\n", file);
        return(EXIT_FAILURE);
    }

    std::cout << "the data are saved to the file: " << filename << "\n";
    return(EXIT_SUCCESS);
}





sigNals readMat(std::string filename){
	/*
	the function readMat reads the mat file "filename" and returns the signals and the MVC
	*/


	MATFile *pmat;			// the object to store and manage the data from mat file

	//	const char **dir;		// the directory of the mat file

	const char *name;		// name of the varables inside the mat file

	//	int ndir;				// length of the direcotry

	mxArray *pa;			// mxArray to store the data

    sigNals EMG;			// struct to store the data

	std::cout << "Reading the file: " << filename << "...\n";

	// open the file

	pmat = matOpen(filename.c_str(), "r");
	if (pmat == NULL){
		std::cout << "Error opening file " << filename << std::endl;

	}


	// store the data of the file to a mxArray
	pa = matGetNextVariable(pmat, &name);
	if (pa == NULL){
		std::cout << " Error reading file " << filename << std::endl;

	}

	std::cout << "The name of the struct inside the file are: " << name << std::endl;

	// get the name of the first variable
	const char *name2 = mxGetFieldNameByNumber(pa, 0);
	//std::cout << "\nThe name of the next struct is: " << name2 << "\n\n";

	// retriev the struct with that name and store it to a mxArray
	mxArray *pa2 = mxGetField(pa, 0, name2);

	// get the number of cells 
	mwSize nElements = (mwSize)mxGetNumberOfElements(pa2);
	//std::cout << "number of objects (cells): " << nElements << std::endl;


	EMG.ob = new eObjects[nElements];			// struct to store the signals

	// for each of the cells of the struct, read and store the matrices
	for (mwIndex eIdx = 0; eIdx < nElements; eIdx++) {

		std::cout << "Object " << eIdx + 1 << "...\n";

		// retrieve the first cell
		mxArray *obj = mxGetCell(pa2, eIdx);
		if (obj == NULL){
			std::cout << "Error\n";
		}

		// get the name of the next struct
		const char *name3 = mxGetFieldNameByNumber(obj, 0);

		//std::cout << "\nThe name of the next struct is: " << name3 << "\n\n";

		// retrieve the next struct
		mxArray *trails = mxGetField(obj, 0, name3);
		if (trails == NULL){
			std::cout << "Error\n";
		}

		// get the number of trials per object
		mwSize nb_trials = (mwSize)mxGetNumberOfElements(trails);
		//std::cout << "number of trials of the object number " << (int)eIdx + 1 << ": " << nb_trials << std::endl;

		for (mwIndex tr = 0; tr < nb_trials; tr++){

			// retrieve the next cell
			mxArray *reach = mxGetCell(trails, tr);

			if ((reach != NULL) && (mxGetClassID(reach) == mxDOUBLE_CLASS) && (!mxIsComplex(reach))){

				// get the number of rows
				size_t nb_rows = mxGetM(reach);

				// get the number of columns
				size_t nb_cols = mxGetN(reach);

				// resize the matrix to store the data
				EMG.ob[eIdx].reach[tr] = std::vector< std::vector<double> >(nb_cols, std::vector<double>(nb_rows, 0));

				//std::cout << "dinemsions of the matrix: " << nb_rows << "x" << nb_cols << "\n";

				// retrieve the data of the matrix and store them to a vector of doubles (concatinated column-wise)
				double *realPtr = mxGetPr(reach);
				if (realPtr != NULL){
					for (int i = 0; i < (int)nb_cols; i++){
						for (int j = 0; j < (int)nb_rows; j++){
							EMG.ob[eIdx].reach[tr][i][j] = realPtr[i*nb_rows + j];
							//std::cout << "(" << i << "," << j << ") = " << realPtr[i*nb_rows + j] << "\n";
						}
					}
				}
			}
		}
	}
	


	// extract the MVC

	std::cout << "extracting MVC ...\n";

	pa = matGetNextVariable(pmat, &name);
	EMG.mvc = std::vector<double>((int)mxGetNumberOfElements(pa), 0);
	double *_mvc = mxGetPr(pa);
	if (_mvc != NULL){
		for (int i = 0; i < (int)EMG.mvc.size(); i++){
			EMG.mvc[i] = _mvc[i];
		}
	}


	// close the file
	if (matClose(pmat) != 0) {
		std::cout << "Error closing file " << filename << std::endl;

	}

	return EMG;
}




int saveData(std::string fname, std::string valName, std::vector< std::vector<double> > Signals){
	MATFile *pmat;
	mxArray *pa;
	fname += ".mat";


	int status;


	double *tmpVec = new double[(int)Signals.size()*Signals[0].size()];//

	
	
	int count = 0;
	for (int j= 0; j<(int)Signals.size(); j++){
		for (int i = 0; i < (int)Signals[j].size(); i++){
			
			tmpVec[count] = Signals[j][i];
			
			count++;		
			
		}
	}


	pa = mxCreateDoubleMatrix(Signals[0].size(), Signals.size(), mxREAL);
	if (pa == NULL) {
		printf("%s : Out of memory on line %d\n", __FILE__, __LINE__);
		printf("Unable to create mxArray.\n");
		return(EXIT_FAILURE);
	}

	std::memcpy((void *)(mxGetPr(pa)), (void *)tmpVec, Signals.size()*Signals[0].size()*sizeof(double));

	

	pmat = matOpen(fname.c_str(), "w");
	if (pmat == NULL) {
		std::cout << "Error creating file: " << fname << "\n";
		std::cout << "(Do you have write permission in this directory?)\n";
		return(EXIT_FAILURE);
	}

    status = matPutVariable(pmat, valName.c_str(), pa);
	if (status != 0) {
		printf("%s :  Error using matPutVariable on line %d\n", __FILE__, __LINE__);
		return(EXIT_FAILURE);
	}



	



	mxDestroyArray(pa);
	if (matClose(pmat) != 0) {
		std::cout << "Error closing file %s\n" << fname << "\n";
		return(EXIT_FAILURE);
	}


	delete tmpVec;

	std::cout << "the data are saved to the file: " << fname << "\n";
	return(EXIT_SUCCESS);


}
