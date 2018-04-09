#ifndef __SEQGENERATOR_H__
#define __SEQGENERATOR_H__

/**
 * \brief Uses an int internal variable to generate sequential integer identifiers.
 * 
 * This class is not thread safe.
 */
class SeqGenerator {
    
    public:

        SeqGenerator(int identifier = 0);
    
    protected:
        /**
         * The internal identifier.
         * Initialized to 0 by default.
         */
        int identifier;

    public:
        /**
         * Returns the next sequential integer identifier.
         */
        int getIdentifier() {
			int prevId = identifier;
			++identifier;

            return prevId;
        }
};

#endif  // __SEQGENERATOR_H__
