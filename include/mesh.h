#ifndef MESH_H
#define MESH_H

#include "tinyply-master/source/tinyply.h"
#include "eigen-3.3.9/Eigen/Geometry"

#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#include <iterator>
#include <istream>


using namespace tinyply;

using namespace Eigen;

/*!
 *  \brief          Mesh class: Utilises tinyply open source framework to read ply files to store mesh data (vertices and faces).
 *  \details        This class takes a const string that defines the location of the ply file to be processed.
 *                  The contents of a ply file are interpreted to store information about vertices, faces, vertex colours,
 *                  texture coordinates and vertex normals. For the purposes of the task, only vertices and faces are stored in private
 *                  class variables. In the context of a triangular mesh, data stored as faces define an index to the three vertex coordinates
 *                  that make up a triangular face. The three vertex coordinates define the triangle in the order of a triangle fan.
 *                  At the bare minimum a ply file will contain information about the vertices and faces of the mesh/3D object.
 *                  The ply file should be structured in the standard ply (Polygon File Format) as defined by its creator Greg Turk.
 *                  Note that some corrupted ply files may include redundant information in the text lines of the ply file that
 *                  could corrupt how tinyply reads the file. Tinyply is robust in that it will throw an error in this instance.
 *                  This may occur when obj files are converted to ply files by an unreliable program.
 *
 *
 *  \author         Patrick Korczak
 *  \version        1.0
 *  \date           2021
 *  \bug            none reported
 *
 */

class Mesh
{

public:

    ///
    /// \brief Mesh class constructor receives a string with the path name to the ply file.
    ///

    Mesh(const std::string& filename)
        : file(filename)
    {
        read_ply_file(file, false);
    }

    ///
    /// \brief getMeshPoints retrieves a vector of Eigen::Vector3f containing the vertices of the mesh
    /// \return Eigen::Vector3f
    ///

   const std::vector<Vector3f> getMeshPoints() const;

   ///
   /// \brief getMeshFaces retrieves a vector of Eigen::Vector3f containing the indexes to the vertices that make up the
   /// triangular faces of the mesh in the order of a triangle fan.
   /// \return Eigen::Vector3f
   ///

   const std::vector<Vector3i> getMeshFaces() const;

private:

    const std::string & file;                           //! Class variable of a string to the path of the ply file assigned in the constructor.

    std::vector<Vector3f> vertices;                     //! Data structure storing the values of vertices

    std::vector<Vector3i> faces;                        //! Data structure storing the indexes to vertices

    ///
    /// \brief setMeshPoints sets the class variable storing the values of vertices
    /// \param p
    ///

    void setMeshPoints(std::vector<Vector3f> &p);

    ///
    /// \brief setMeshFaces sets the class variable storing the indexes to vertices
    /// \param f
    ///

    void setMeshFaces(const std::vector<Vector3i> &f);

    ///
    /// \brief read_ply_file tinyply function used to pull out data that was read by tinyply into Mesh variables
    /// \param filepath
    /// \param preload_into_memory
    ///

    void read_ply_file(const std::string & filepath, const bool preload_into_memory);

    ///
    /// \brief read_file_binary inline function used to read binary ply files
    /// \param pathToFile
    /// \return
    ///


    inline std::vector<uint8_t> read_file_binary(const std::string & pathToFile)
    {
        std::ifstream file(pathToFile, std::ios::binary);
        std::vector<uint8_t> fileBufferBytes;

        if (file.is_open())
        {
            file.seekg(0, std::ios::end);
            size_t sizeBytes = file.tellg();
            file.seekg(0, std::ios::beg);
            fileBufferBytes.resize(sizeBytes);
            if (file.read((char*)fileBufferBytes.data(), sizeBytes)) return fileBufferBytes;
        }
        else throw std::runtime_error("could not open binary ifstream to path " + pathToFile);
        return fileBufferBytes;
    }

    struct memory_buffer : public std::streambuf
    {
        char * p_start {nullptr};
        char * p_end {nullptr};
        size_t size;

        memory_buffer(char const * first_elem, size_t size)
            : p_start(const_cast<char*>(first_elem)), p_end(p_start + size), size(size)
        {
            setg(p_start, p_start, p_end);
        }

        pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
        {
            if (dir == std::ios_base::cur) gbump(static_cast<int>(off));
            else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
            return gptr() - p_start;
        }

        pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
        {
            return seekoff(pos, std::ios_base::beg, which);
        }
    };

    struct memory_stream : virtual memory_buffer, public std::istream
    {
        memory_stream(char const * first_elem, size_t size)
            : memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf*>(this)) {}
    };

    class manual_timer
    {
        std::chrono::high_resolution_clock::time_point t0;
        double timestamp{ 0.0 };
    public:
        void start() { t0 = std::chrono::high_resolution_clock::now(); }
        void stop() { timestamp = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count() * 1000.0; }
        const double & get() { return timestamp; }
    };

};

#endif // MESH_H
