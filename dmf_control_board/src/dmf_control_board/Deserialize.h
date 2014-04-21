#ifndef ___DESERIALIZE__HPP___
#define ___DESERIALIZE__HPP___

#include <stdint.h>
#if !( defined(AVR) || defined(__SAM3X8E__) )
  #include <boost/format.hpp>
#endif


template <typename T> struct is_numeric {};
template <> struct is_numeric<uint8_t>  { static const bool value = true; };
template <> struct is_numeric<int8_t> { static const bool value = true; };
template <> struct is_numeric<uint16_t> { static const bool value = true; };
template <> struct is_numeric<int16_t> { static const bool value = true; };
template <> struct is_numeric<uint32_t> { static const bool value = true; };
template <> struct is_numeric<int32_t> { static const bool value = true; };
template <> struct is_numeric<float> { static const bool value = true; };
template <> struct is_numeric<double> { static const bool value = true; };
template <> struct is_numeric<const char *> { static const bool value = false; };

template <typename T> struct is_signed {};
template <> struct is_signed<uint8_t>  { static const bool value = false; };
template <> struct is_signed<int8_t> { static const bool value = true; };
template <> struct is_signed<uint16_t> { static const bool value = false; };
template <> struct is_signed<int16_t> { static const bool value = true; };
template <> struct is_signed<uint32_t> { static const bool value = false; };
template <> struct is_signed<int32_t> { static const bool value = true; };
template <> struct is_signed<float> { static const bool value = true; };
template <> struct is_signed<double> { static const bool value = true; };
template <> struct is_signed<const char *> { static const bool value = false; };

template <typename T> struct bit_width {};
template <> struct bit_width<uint8_t> { static const int value = 8; };
template <> struct bit_width<int8_t> { static const int value = 8; };
template <> struct bit_width<uint16_t> { static const int value = 16; };
template <> struct bit_width<int16_t> { static const int value = 16; };
template <> struct bit_width<uint32_t> { static const int value = 32; };
template <> struct bit_width<int32_t> { static const int value = 32; };
template <> struct bit_width<float> { static const int value = 0; };
template <> struct bit_width<double> { static const int value = 1; };
template <> struct bit_width<const char *> { static const int value = 2; };


template <typename Buffer, typename T>
inline T deserialize(Buffer buffer, T &result) {
  result = *(T *)(buffer);
  return sizeof(T);
}


#if !( defined(AVR) || defined(__SAM3X8E__) )
template <typename T>
inline std::string type_format() {
  /* Return the format string to print out a value of type `T` using `printf`.
   * */
  if (bit_width<T>::value == 0 && bit_width<T>::value == 1) {
    /* `float` or `double` */
    return "%f";
  } else if (is_numeric<T>::value) {
    /* `integer` */
    return "%d";
  } else {
    /* `const char *` */
    return "%s";
  }
}


template <typename T>
inline std::string type_label() {
  /* Return the name of a type as a `std::string`.  For example, if `T` is an
   * 8-bit unsigned integer, the label will be `uint8_t`. */
  if (bit_width<T>::value == 0) {
    return "float";
  } else if (bit_width<T>::value == 1) {
    return "double";
  } else if (is_numeric<T>::value) {
    size_t bits = bit_width<T>::value;
    std::string base_str;
    if (is_numeric<T>::value) {
      base_str = "uint";
    } else {
      base_str = "int";
    }
    return str(boost::format("%s%d_t") % base_str % bits);
  } else {
    return "string";
  }
}
#endif  // !( defined(AVR) || defined(__SAM3X8E__) )

#endif
