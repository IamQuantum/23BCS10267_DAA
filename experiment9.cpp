class Solution {
  public:
    vector<int> search(string &pat, string &txt) {
        int n = txt.size(), m = pat.size();
        vector<int> lps(m, 0);
        vector<int> ans;

        // Step 1: Build LPS array
        int len = 0, i = 1;
        while (i < m) {
            if (pat[i] == pat[len]) {
                len++;
                lps[i] = len;
                i++;
            } else {
                if (len != 0) {
                    len = lps[len - 1];
                } else {
                    lps[i] = 0;
                    i++;
                }
            }
        }

        // Step 2: Search pattern in text using LPS
        i = 0;  // index for txt
        int j = 0;  // index for pat
        while (i < n) {
            if (txt[i] == pat[j]) {
                i++;
                j++;
            }

            if (j == m) {
                ans.push_back(i - j); // pattern found
                j = lps[j - 1];
            } else if (i < n && txt[i] != pat[j]) {
                if (j != 0)
                    j = lps[j - 1];
                else
                    i++;
            }
        }

        return ans;
    }
};