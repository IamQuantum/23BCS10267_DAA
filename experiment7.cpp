class Solution {
  public:
    int knapsack(int W, vector<int> &val, vector<int> &wt) {
        int n = val.size();
        vector<vector<int>> dp(n + 1, vector<int>(W + 1, 0));

        // Build table dp[][] bottom up
        for (int i = 1; i <= n; i++) {
            for (int w = 1; w <= W; w++) {
                // If weight of current item is less than or equal to capacity
                if (wt[i - 1] <= w) {
                    dp[i][w] = max(
                        val[i - 1] + dp[i - 1][w - wt[i - 1]],  // include item
                        dp[i - 1][w]                            // exclude item
                    );
                } else {
                    dp[i][w] = dp[i - 1][w]; // can't include item
                }
            }
        }
        return dp[n][W]; // maximum value with all items and full capacity
    }
};