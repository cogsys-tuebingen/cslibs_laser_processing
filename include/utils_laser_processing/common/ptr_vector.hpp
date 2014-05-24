#ifndef PTR_VECTOR_HPP
#define PTR_VECTOR_HPP
#include <boost/shared_ptr.hpp>
#include "vector.hpp"

namespace vector {
/**
 * @brief Do a cleanup on a list of pointers.
 * @param Tp        the typename
 * @param pointers  vector of pointers to delete
 */
template<typename Tp>
inline void cleanupPointers(std::vector<Tp*> &pointers)
{
    for(typename std::vector<Tp*>::iterator it = pointers.begin() ; it != pointers.end() ; ++it)
        delete *it;

    pointers.clear();
}

template<typename Tp>
inline void clonePointers(const std::vector<Tp*> &src, std::vector<Tp*> &dst)
{
    for(typename std::vector<Tp*>::const_iterator it = src.begin() ; it != src.end() ; ++it) {
        dst.push_back(new Tp(*it));
    }
}

template<typename Tp>
inline void cloneSharedPointers(const std::vector<boost::shared_ptr<Tp> > &src,
                                std::vector<boost::shared_ptr<Tp> > &dst)
{
    for(typename std::vector<boost::shared_ptr<Tp> >::const_iterator it = src.begin() ; it != src.end() ; ++it) {
        dst.push_back(boost::shared_ptr<Tp>(new Tp(*(*it)->get())));
    }
}

}
#endif // PTR_VECTOR_HPP
