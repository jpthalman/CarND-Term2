//
// Created by japata on 4/17/17.
//

#ifndef UNSCENTEDKALMANFILTER_NOTIMPLEMENTEDEXCEPTION_H
#define UNSCENTEDKALMANFILTER_NOTIMPLEMENTEDEXCEPTION_H


#include <exception>
#include <string>

class NotImplementedException : public std::exception {
public:
    // construct with given error message
    NotImplementedException(const char *error = "Functionality not yet implemented.") {
        error_message = error;
    }

    // for compatibility with std::exception
    const char* what() const noexcept {
        return error_message.c_str();
    }

private:
    std::string error_message;
};


#endif //UNSCENTEDKALMANFILTER_NOTIMPLEMENTEDEXCEPTION_H
