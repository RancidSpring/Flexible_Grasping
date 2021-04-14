import sys


def writeVector(q):
    """Writes a vector to text 'n v1 ... vn'"""
    return str(len(q))+'\t'+' '.join(str(v) for v in q)


def readVector(text):
    """Reads a vector from text 'n v1 ... vn'"""
    items = text.split()
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return [float(v) for v in items[1:]]


def openhand(config, hand, amount, index_arr):
    """
    Opens the hand of the given Hubo configuration config by the given amount.
    hand is either 'l','r', 'left', or 'right' and amount is in the range [0,1]
    ranging from closed to open.
    """
    if hand != 'kuka':
        raise ValueError("Invalid hand specified, must be kuka")
    dofs = range(15, 28)
    open = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    close = [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    output = config[:]
    vals = []
    i = 0
    for a, b in list(zip(close, open)):
        if i+15 not in index_arr:
            vals.append(a + amount * (b - a))
        else:
            vals.append(b)
        i += 1
    for (d, v) in list(zip(dofs, vals)):
        assert d < len(config), "Config does not have enough entries"
        output[d] = v
    return output

def open_free(config, hand, amount):
    """
        Opens the hand of the given Hubo configuration config by the given amount.
        hand is either 'l','r', 'left', or 'right' and amount is in the range [0,1]
        ranging from closed to open.
        """
    if hand != 'kuka':
        raise ValueError("Invalid hand specified, must be kuka")
    dofs = range(15, 28)
    open = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    close = [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    output = config[:]
    vals = []
    for a, b in list(zip(close, open)):
        vals.append(a + amount * (b - a))
    for (d, v) in list(zip(dofs, vals)):
        assert d < len(config), "Config does not have enough entries"
        output[d] = v
    return output

if __name__ == '__main__':
    import optparse
    usage = "Usage: %prog [options] hand amount"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-i", "--in", dest="infile",
                      help="read config from FILE", metavar="FILE")
    parser.add_option("-o", "--out", dest="outfile",
                      help="write config to FILE", metavar="FILE")
    (options,args) = parser.parse_args()

    infile = options.infile
    outfile = options.outfile

    hand = args[0].lower()
    amount = 1
    try:
        amount = float(args[1])
    except:
        print("Amount needs to be a numeric value")
        exit(1)

    config = []
    if infile:
        f = open(infile,'r')
        lines = ' '.join(f.readlines())
        config = readVector(lines)
    else:
        lines = ' '.join(sys.stdin.readlines())
        config = readVector(lines)

    config = openhand(config, hand, amount)

    if outfile:
        f = open(outfile, 'w')
        f.write(writeVector(config))
        f.write('\n')
    else:
        print(writeVector(config))

