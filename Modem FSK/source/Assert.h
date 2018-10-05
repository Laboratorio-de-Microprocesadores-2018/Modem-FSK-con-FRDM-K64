/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file Assert.h
 * @brief Define assertion macro to halt the program execution.
 *
 * It halts the program and stores the filename and line where the assertion has
 * failed.
 */

#ifndef ASSERT_H_
#define ASSERT_H_

void assertFailed(char *file, int line,char*expr);

#define ASSERT(expr) \
		 if (expr){} \
		 else assertFailed(__FILE__, __LINE__,#expr)

#endif
