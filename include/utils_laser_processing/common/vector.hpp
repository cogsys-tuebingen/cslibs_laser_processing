#ifndef VECTOR_HPP
#define VECTOR_HPP
#include <vector>
namespace vector {
/**
 * @brief Copy a vector with padding in the beginning and the end of a vector.
 *        The padding will be filled with a fixed value.
 * @param Tp        the typename
 * @param input     the vector to copy
 * @param value     the padding value
 * @param padding   the padding size
 * @param output    the output
 */
template<typename Tp>
inline void copyWithPadding(const std::vector<Tp> &input, const Tp value, const unsigned int padding, std::vector<Tp> &output)
{
    output.clear();
    output.resize(input.size() + 2 * padding, value);
    std::copy(input.begin(), input.end(), output.begin() + padding);
}

/**
 * @brief Add padding to the front of a vector.
 * @param Tp        the typename
 * @param value     the value the padding will be filled with
 * @param padding   the padding size
 * @param input     the vector to add padding to
 */
template<typename Tp>
inline void addFrontPadding(const Tp value, const unsigned int padding, std::vector<Tp> &input)
{
    input.insert(input.begin(),padding, value);
}

/**
 * @brief Add padding to the back of a vector.
 * @param Tp        the typename
 * @param value     the value the padding will be filled with
 * @param padding   the padding size
 * @param input     the vector to add padding to
 */
template<typename Tp>
inline void addBackPadding(const Tp value, const unsigned int padding, std::vector<Tp> &input)
{
    for(unsigned int i = 0 ; i < padding ; ++i)
        input.push_back(value);
}

/**
 * @brief The 'convertType' enables the possiblity to convert vector types.
 * @param Tp        input type
 * @param Sp        output type
 * @param input     the input vector
 * @param output    the output vector
 */
template<typename Tp, typename Sp>
inline void convertType(const std::vector<Tp> &input_ranges, std::vector<Sp> &output_ranges)
{
    output_ranges.clear();
    for(typename std::vector<Tp>::const_iterator it = input_ranges.begin() ; it != input_ranges.end() ; ++it)
        output_ranges.push_back(*it);
}
}

#endif // VECTOR_HPP
