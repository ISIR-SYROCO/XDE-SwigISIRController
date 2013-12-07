// typemap_eigen_lgsm.i

%include "numpy.i"
%init %{
    import_array();
%}



%{

static PyObject* py_lgsm_displacement;
static PyObject* py_lgsm_twist;
static PyObject* py_lgsm_quaternion;
static PyObject* py_numpy_asmatrix;


std::vector<std::string> split_string(const std::string& str, const char& ch) {
    std::string next;
    std::vector<std::string> result;

    // For each character in the string
    for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
         result.push_back(next);
    return result;
}



void initialize_py_lgsm_functions()
{
    PyObject* py_numpy_module = PyImport_ImportModule("numpy");
    if (py_numpy_module == NULL)
    {
        std::cout<<"Error: cannot load 'numpy' which is a required module;\n";
        return;
    }

    PyObject* py_lgsm_module  = PyImport_ImportModule("lgsm");
    if (py_lgsm_module == NULL)
    {
        PyErr_Clear(); // Clear the error set
        std::cout<<"Warning: cannot load 'lgsm' (have you run with xde?); try to find it using XDE_PATH or PATH\n";
        char* pPath;
        std::string xdeStrPath("");
        pPath = getenv("XDE_PATH");
        if (pPath!=NULL)
        {
            std::string xdeStrPath(pPath);
        }
        else
        {
            std::cout<<"XDE_PATH does not exist; try to find xde in PATH\n";
            pPath = getenv("PATH");
            if (pPath==NULL)
            {
                std::cout<<"PATH does not exist\n";
            }
            else
            {
                std::string strPath(pPath);
                std::vector<std::string> vectPath = split_string(strPath, ':');
                for (unsigned int i=0; i< vectPath.size(); i++)
                {
                    if (vectPath[i].size() >= 4)
                    {
                        if (vectPath[i].substr(vectPath[i].size()-4) == "xde/")
                        {
                            std::cout<<"XDE found!! ==> "<<vectPath[i]<<"\n";
                            xdeStrPath = vectPath[i];
                            break;
                        }
                        if (vectPath[i].substr(vectPath[i].size()-3) == "xde")
                        {
                            std::cout<<"XDE found!! ==> "<<vectPath[i]<<"\n";
                            xdeStrPath = vectPath[i]+"/";
                            break;
                        }
                    }
                }
            }
        }
        
        if (xdeStrPath != "")
        {
            std::string lgsmStrPath =  xdeStrPath + "bin/lib/modules/base/math/src/pyLgsm";
            std::string cmd = std::string("import sys;sys.path.append('")+lgsmStrPath+"')";
            PyRun_SimpleString(cmd.c_str());
            py_lgsm_module  = PyImport_ImportModule("lgsm"); // retry to load lgsm
        }
    }
    
    if (py_lgsm_module == NULL)
    {
        std::cout<<"Warning: cannot load 'lgsm' even by using XDE_PATH or PATH; Displacement, Twist & Quaternion will be expressed as matrix\n";
        PyErr_Clear(); // Clear the error set
        
        PyObject *py_main_module, *py_main_dict;
        py_main_module = PyImport_AddModule("__main__");
        py_main_dict   = PyModule_GetDict(py_main_module);
        std::string AllClassSrc("import numpy\nclass Displacement:\n  def __init__(self,x,y,z,qw,qx,qy,qz):\n    self.d = numpy.array([qx,qy,qz,qw,x,y,z])\n  def __repr__(self):\n    return ' '.join([str(self.d[i]) for i in [4,5,6,3,0,1,2]])\n");
        AllClassSrc += "def Twist(rx,ry,rz,vx,vy,vz):\n  return numpy.array([rx,ry,rz,vx,vy,vz])\n";
        AllClassSrc += "def Quaternion(qw,qx,qy,qz):\n  return numpy.array([qw,qx,qy,qz])\n";
        PyRun_String(AllClassSrc.c_str(), Py_file_input, py_main_dict, py_main_dict);
        //PyRun_SimpleString("\n");
        py_lgsm_displacement = PyObject_GetAttrString(py_main_module, "Displacement");
        py_lgsm_twist        = PyObject_GetAttrString(py_main_module, "Twist");
        py_lgsm_quaternion   = PyObject_GetAttrString(py_main_module, "Quaternion");
        py_numpy_asmatrix    = PyObject_GetAttrString(py_numpy_module, "asmatrix");
        Py_DECREF(py_main_module);
    }
    else
    {
        py_lgsm_displacement = PyObject_GetAttrString(py_lgsm_module , "Displacement");
        py_lgsm_twist        = PyObject_GetAttrString(py_lgsm_module , "Twist");
        py_lgsm_quaternion   = PyObject_GetAttrString(py_lgsm_module , "Quaternion");
        py_numpy_asmatrix    = PyObject_GetAttrString(py_numpy_module, "asmatrix");
        Py_DECREF(py_lgsm_module);
    }
    Py_DECREF(py_numpy_module);

}

%}



%init %{
    initialize_py_lgsm_functions();
%}



//----------------------------------------------------------------------------//
// Definition of TYPEMAP
//----------------------------------------------------------------------------//

%typemap(out) Eigen::Displacementd {
    PyObject* arglist = Py_BuildValue("(ddddddd)", $1.x(), $1.y(), $1.z(), $1.qw(), $1.qx(), $1.qy(), $1.qz() );
    $result = PyObject_CallObject(py_lgsm_displacement, arglist);
    Py_DECREF(arglist);
}
%typemap(out) Eigen::Displacementd& {
    PyObject* arglist = Py_BuildValue("(ddddddd)", $1->x(), $1->y(), $1->z(), $1->qw(), $1->qx(), $1->qy(), $1->qz() );
    $result = PyObject_CallObject(py_lgsm_displacement, arglist);
    Py_DECREF(arglist);
}

%typemap(out) Eigen::Twistd {
    PyObject* arglist = Py_BuildValue("(dddddd)", $1.rx(), $1.ry(), $1.rz(), $1.vx(), $1.vy(), $1.vz() );
    $result = PyObject_CallObject(py_lgsm_twist, arglist);
    Py_DECREF(arglist);
}
%typemap(out) Eigen::Twistd& {
    PyObject* arglist = Py_BuildValue("(dddddd)", $1->rx(), $1->ry(), $1->rz(), $1->vx(), $1->vy(), $1->vz() );
    $result = PyObject_CallObject(py_lgsm_twist, arglist);
    Py_DECREF(arglist);
}

%typemap(out) Eigen::Rotation3d {
    PyObject* arglist = Py_BuildValue("(dddd)", $1.w(), $1.x(), $1.y(), $1.z() );
    $result = PyObject_CallObject(py_lgsm_quaternion, arglist);
    Py_DECREF(arglist);
}
%typemap(out) Eigen::Rotation3d& {
    PyObject* arglist = Py_BuildValue("(dddd)", $1->w(), $1->x(), $1->y(), $1->z() );
    $result = PyObject_CallObject(py_lgsm_quaternion, arglist);
    Py_DECREF(arglist);
}

%typemap(out) Eigen::MatrixXd& {
    npy_intp dims[2];
    dims[0] = (npy_intp) $1->rows();
    dims[1] = (npy_intp) $1->cols();
    PyObject* res_array = PyArray_EMPTY(2, dims, NPY_DOUBLE,1) ;

    char* res_array_data = PyArray_BYTES( reinterpret_cast< PyArrayObject* >(res_array) );
    memcpy( res_array_data, $1->data(), sizeof(double)*$1->size() );

    PyObject* arglist = Py_BuildValue("(O)", res_array );
    $result = PyObject_CallObject(py_numpy_asmatrix, arglist);
    Py_DECREF(arglist);
}



%apply Eigen::MatrixXd& { Eigen::VectorXd& };
%apply Eigen::MatrixXd& { Eigen::Vector3d& };
%apply Eigen::MatrixXd& { Eigen::Matrix<double,3,Eigen::Dynamic>& };
%apply Eigen::MatrixXd& { Eigen::Matrix<double,6,Eigen::Dynamic>& };
%apply Eigen::MatrixXd& { Eigen::Matrix<double,6,6>& };



%typemap(in) Eigen::VectorXi& {
    PyObject* arr = reinterpret_cast< PyObject* >($input); // no 'PyArrayObject' because it can also be a list, typle or whatever...

    int size = PyList_Size(arr);
    if (size > 0)   // --> array is a list
    {
        $1 = new Eigen::VectorXi(size);
        for (int i=0; i<size; ++i)
        {
            (*$1)(i) = static_cast< int >(PyInt_AsLong(PyList_GetItem(arr, i)) );
        }
    }
    else            // --> then it should be a numpy array/matrix
    {
        throw std::runtime_error("[typemap(in) Eigen::VectorXi] : USE PYTHON LIST; do not use numpy array/matrix as python input argument for Eigen::MatrixXi c++ input arg");
    }
}
%typemap(freearg) Eigen::VectorXi& {
   delete $1;
}

%typemap(in) Eigen::VectorXd& {
    PyArrayObject* arr = reinterpret_cast< PyArrayObject* >($input);

    int arr_size=0;
    if (arr->nd == 1)
        arr_size = arr->dimensions[0];
    else if (arr->nd == 2)
        arr_size = arr->dimensions[0]*arr->dimensions[1];

    $1 = new Eigen::VectorXd(arr_size);
    char* res_array_data = PyArray_BYTES( arr );
    memcpy( $1->data(), res_array_data, sizeof(double)*arr_size );
}
%typemap(freearg) Eigen::VectorXd& {
   delete $1;
}



%typemap(in) Eigen::MatrixXd& {
    PyArrayObject* arr = reinterpret_cast< PyArrayObject* >($input);

    int arr_row  = 0;
    int arr_col  = 0;
    int arr_size = 0;
    if (arr->nd != 2)
        throw std::runtime_error("[typemap(in) Eigen::MatrixXd] : input argument is not a numpy array of dim 2");
    else
    {
        arr_row  = arr->dimensions[1]; //Because numpy is RowMajor and Eigen is ColMajor
        arr_col  = arr->dimensions[0];
        arr_size = arr_row * arr_col;
    }

    $1 = new Eigen::MatrixXd(arr_row, arr_col);
    char* res_array_data = PyArray_BYTES( arr );
    memcpy( $1->data(), res_array_data, sizeof(double)*arr_size );
    $1->transposeInPlace(); //Because numpy is RowMajor and Eigen is ColMajor
}
%typemap(freearg) Eigen::MatrixXd& {
   delete $1;
}


%typemap(in) Eigen::Displacementd& {
    PyArrayObject* arr = reinterpret_cast< PyArrayObject* >( PyObject_GetAttrString($input , "d") );
    double* res_array_data = (double *) PyArray_DATA( arr );
    $1 = new Eigen::Displacementd;
    $1->x()  = res_array_data[4];
    $1->y()  = res_array_data[5];
    $1->z()  = res_array_data[6];
    $1->qw() = res_array_data[3];
    $1->qx() = res_array_data[0];
    $1->qy() = res_array_data[1];
    $1->qz() = res_array_data[2];
}
%typemap(freearg) Eigen::Displacementd& {
   delete $1;
}


%typemap(in) Eigen::Twistd& {
    PyArrayObject* arr = reinterpret_cast< PyArrayObject* >( $input );
    double* res_array_data = (double *) PyArray_DATA( arr );
    $1 = new Eigen::Twistd;
    $1->rx() = res_array_data[0];
    $1->ry() = res_array_data[1];
    $1->rz() = res_array_data[2];
    $1->vx() = res_array_data[3];
    $1->vy() = res_array_data[4];
    $1->vz() = res_array_data[5];
}
%typemap(freearg) Eigen::Twistd& {
   delete $1;
}

%typemap(in) Eigen::Wrenchd& {
    PyArrayObject* arr = reinterpret_cast< PyArrayObject* >( $input );
    double* res_array_data = (double *) PyArray_DATA( arr );
    $1 = new Eigen::Wrenchd;
    $1->tx() = res_array_data[0];
    $1->ty() = res_array_data[1];
    $1->tz() = res_array_data[2];
    $1->fx() = res_array_data[3];
    $1->fy() = res_array_data[4];
    $1->fz() = res_array_data[5];
}
%typemap(freearg) Eigen::Wrenchd& {
   delete $1;
}


// THE FOLLOWING IS FOR TYPECHECK USE FOR TYPEMAP & OVERLOADING. ACTUALLY NOTHING DONE.
%typecheck(1200) Eigen::VectorXd& {
    $1 = 1;
}
%typecheck(1201) Eigen::Displacementd& {
    $1 = 1;
}
%typecheck(1202) Eigen::Twistd& {
    $1 = 1;
}

%typecheck(1203) Eigen::VectorXi& {
    $1 = 1;
}
