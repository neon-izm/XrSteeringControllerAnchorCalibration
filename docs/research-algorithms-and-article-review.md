# アルゴリズム改善リサーチ & Zenn 記事整合性レビュー

対象リポジトリ: `XrSteeringControllerAnchorCalibration`  
対象記事: [UnityでハンドルコントローラをCGの座標系に一致させる方法](https://zenn.dev/izm/articles/b98535429ea1c9)  
調査日: 2026-07-09

---

## 1. 現状パイプライン（コード準拠）

```
手の軌跡点 (world)
  → MSAC (3点サンプル → 円候補 → truncated quadratic score)
  → inlier 抽出 (threshold 既定 5mm)
  → PCA で平面推定 → 2D 代数フィット (Kåsa / Delogne–Kåsa)
  → Circle3D (roll=0)
  → AlignNormalToTargetForward (CG forward 半球)
  → 剛体 ModelView (scale=1)
```

主要実装:

| 段階 | ファイル | 要点 |
|------|----------|------|
| MSAC | `CircleFitting3D.FitCircleMsac` | 3点最小サンプル、スコアは `min(d², τ²)` の和 |
| 精密化 | `RefineWithInliers` | 共分散 PCA → 平面投影 → `FitCircle2D` |
| 2D フィット | `FitCircle2D` | 代数距離最小化（Kåsa 型） |
| 姿勢合わせ | `AnchorCalibration.ComputeModelView` | `R = R_cg * R_est⁻¹`, `t = p_cg - R * p_est` |

---

## 2. より良い実装案・アルゴリズム

優先度は「このユースケース（約90°の短い円弧 + 数%外れ値 + 数mmノイズ）」への効きやすさ順。

### A. 精密化の 2D フィットを Taubin / Pratt / Geometric に置換（高優先）

**現状の弱点:** `FitCircle2D` は古典的な代数フィット（Kåsa）。Chernov らの誤差解析では、**短い円弧では半径を過小推定する強いバイアス**が知られている。

本パッケージのサンプル・テストも **±45°（合計90°）の部分弧**を生成しており、記事の「90度くらい回す」と一致する。つまり Kåsa バイアスが最も出やすい条件で運用している。

| 手法 | 性質 | 短い弧での傾向 | 実装コスト |
|------|------|----------------|------------|
| Kåsa（現状） | 代数・閉形式 | 半径過小バイアス大 | 済 |
| Pratt | 代数・閉形式 | 中程度 | 小 |
| **Taubin** | 代数・閉形式 | バイアス小、安定 | 小〜中 |
| Geometric (Gander 等) | 幾何距離の反復最小化 | 短弧で最良クラス | 中（初期値に Taubin 推奨） |
| Hyperaccurate / HyperLS | 代数・バイアス補正 | 長弧で特に強い | 中 |

**推奨:** MSAC の仮説生成はそのまま、`RefineWithInliers` 内だけを **Taubin →（任意で）geometric refine** に差し替える。API 互換を保てる。

参考文献:

- Al-Sharadqah & Chernov, *Error analysis for circle fitting algorithms*, Electron. J. Statist. 2009
- Al-Sharadqah & Chernov, *Further statistical analysis of circle fitting*, Electron. J. Statist. 2015
- Chernov の公開実装メモ（Kåsa / Pratt / Taubin / geometric の比較）

### B. 平面推定を robust にする（中〜高優先）

現状は inlier 集合に対する通常 PCA。外れ値が残ると法線が傾き、円中心もずれる。

候補:

1. **MSAC/RANSAC で平面を先に推定** → 投影 → 2D 円フィット（Nurunnabi et al., Pattern Recognition 2018 の流れに近い）
2. **LMedS / LTS** による robust PCA
3. 反復: フィット → 再 inlier → 再フィット（1〜2回で十分なことが多い）

ハンドトラッキングの外れ値は「散在」より「瞬間的なジャンプ」が多いので、現状 MSAC でも実用域だが、クラスタ状外れ値には B が効く。

### C. MSAC ループの実装改善（中優先・バグ寄り）

`FitCircleMsac` の適応イテレーション更新:

```csharp
if (adaptiveIterations > iterations)
{
    iterations = adaptiveIterations;
}
```

標準 RANSAC/MSAC では、良いモデルが見つかると **必要試行回数は減る**（`N = log(1-p)/log(1-w³)`）。現状は「増やす方向にしか更新しない」ため、外れ値が多いときに正しく増える一方、良いモデル発見後の早期打ち切りが効かない。

加えて:

- 初期 `EstimateIterationCount(0.9f, …)` は楽観的（外れ値15%想定なら数百〜数千回が妥当）
- 同一インデックスの3点が `continue` されるだけで、**重複なしサンプリング**や **stratified** ではない
- スコア最良時だけ inlier を再収集しているが、最終 refine 後に **再スコア / 再 inlier** していない

**推奨:** 適応更新を `iterations = min(iterations, adaptive)`（上限付き）に直し、refine 後に1回 re-estimate inliers する。

### D. 仮説生成の質を上げる（中優先）

3点円は短弧・近接点だと数値的に不安定。改善案:

- 3点間の最小距離・角度に下限を設ける（degenerate 棄却）
- まず平面を MSAC で取り、平面上で 2D 3点円を仮説にする（次元削減で安定）
- Progressive Sample Consensus (PROSAC): 時系列で近い点を優先（軌跡データ向き）

### E. 幾何モデルの拡張（低〜中・プロダクト次第）

現状は「点の軌跡 → 円」のみ。実運用では:

| 拡張 | 効果 |
|------|------|
| 手首/掌のオフセットを既知として円半径を拘束 | 自由度↓、短弧でも安定 |
| 両手同時弧（左右グリップ） | 中心・法線の冗長観測 |
| 時系列（角速度一定の円弧運動）を運動モデルに入れる | 外れ値判定が楽 |
| 既知の物理ハンドル半径がある場合は `ComputeModelViewWithScale` を正式 API 化 | スケール誤差を吸収 |

`ComputeModelViewWithScale` は既に存在するが、記事・README の主経路からは外れている（意図的）。

### F. 法線符号の曖昧性（現状の制約は妥当、改善余地あり）

記事・README どおり「CG forward 半球」で解を一意化している。追加案:

- キャリブ中に「手前を握る」などユーザー指示で、視線方向との内積を使う
- 弧の進行方向（右手系）から回転軸の向きを決める（時系列が必要）
- 完全水平軸は記事どおり未対応のまま明示でよい

### G. 「より良い」が過剰になりやすい案（非推奨 or 後回し）

- フルバンドル調整で ModelView を直接最適化: 円フィット分離の方が解釈・デバッグしやすい
- 深層学習ベースの円検出: この問題規模では過剰
- Vive Tracker 併用: 記事の動機（トラッカー不要）と逆

---

## 3. 現状実装の妥当性（結論）

**方針（MSAC → inlier refine → 剛体 ModelView）は用途に対して妥当。**  
外れ値耐性と「半径未知・roll=0」の制約設計は筋が良い。

一方で、**精密化が Kåsa のままなのは短弧キャリブとして最大の技術的弱点**。ここを Taubin（必要なら geometric）に替えるのが、コスト対効果で最も高い改善。

---

## 4. Zenn 記事 vs README / コードの整合性

### 一致している点（矛盾なし）

| 主張 | 記事 | コード / README |
|------|------|-----------------|
| 手の部分円弧から 3D 円を推定 | ○ | `FitCircleMsac` |
| 外れ値耐性に MSAC | ○（手順の説明も実装と一致） | `score += min(d², τ²)` |
| 閾値デフォルト約 5mm | ○ | `DefaultThreshold = 0.005f` |
| 3点で円候補 | ○ | `CircleFrom3Points` |
| 最後に inlier で精密化 | ○ | `RefineWithInliers` |
| roll=0、forward=法線 | ○ | `EnforceZeroRoll` / `Circle3D` |
| CG 半径は未知、剛体のみ | ○ | `CgHandlePose` に半径なし、`scale=1` |
| CG 姿勢は既知 | ○ | `Calibrate(..., targetHandle)` |
| 法線 ± は CG forward 半球で解決 | ○ | `AlignNormalToTargetForward` |
| 完全水平軸は曖昧で未対応 | ○ | dot≈0 なら反転しない |
| API 例 `CgHandlePose` + `Calibrate` → `ModelView` | ○ | README / 記事とも一致 |
| サンプルを Package Manager から Import | ○ | `package.json` samples |

記事の数式イメージ `CG上の点 = ModelView × トラッキング上の点` も、コードの  
`modelView.MultiplyPoint3x4(worldPoint)` と一致する。

### 軽微な食い違い・省略（致命的な矛盾ではない）

1. **「精密化」の中身**  
   記事は「良さそうな点だけ集めてもう一度キレイに円フィット」とだけ書く。実装は PCA + **Kåsa 代数フィット**。誤りではないが、読者が「幾何フィット」と誤解しうる。

2. **MSAC の適応イテレーション**  
   記事の「何百回も繰り返す」は概念説明として正しい。実装は初期50回＋適応更新だが、前述のとおり更新方向が標準と逆。記事の説明自体は矛盾ではない。

3. **`ComputeModelViewWithScale`**  
   コードにはスケール付き API があるが、記事・README の主ストーリーは「スケールなし」。意図的なスコープ制限で、矛盾というよりドキュメント未言及。

4. **Unity バージョン**  
   記事は触れていない。README は Unity 6（6000.3+推奨）、`package.json` は `"unity": "6000.0"`。記事との矛盾ではなく、README 内の推奨と package 宣言の差。

5. **「90度くらい」**  
   記事の体験談と、サンプル生成の ±45° は整合。必須条件としてはコード上ハードコードされていない（任意点群を受け取る）。

6. **用語「ModelView」**  
   コンピュータグラフィックスの古典的 model-view（世界→カメラ）とは意味が違い、ここでは「トラッキング→CG」の剛体変換。記事・README・コードで一貫して同じ意味なので内部矛盾はないが、外部読者には注釈があると親切。

### 記事にあってコードに明示がない／弱い点

- 「たまにトラッキングが飛ぶ」への対策は MSAC のみ（時系列フィルタや速度ゲートはなし）
- 「机マウントがちょっと傾いているくらいなら大抵 OK」はヒューリスティック説明で、定量テストは水平近傍の専用ケースがない

### 総合判定

**記事とリポジトリの主張に、事実と食い違う矛盾は見当たらない。**  
アルゴリズムの骨格・制約・API・デフォルト閾値は一致している。  
改善するなら記事側は「精密化 = 代数（将来 Taubin/geometric）」の一言、コード側は短弧向けフィットの置換が主。

---

## 5. 推奨ロードマップ（実装するなら）

1. `FitCircle2D` を Taubin に置換（または切替可能に）し、短弧テストを追加  
2. MSAC の適応イテレーションを標準形に修正 + refine 後の re-inlier  
3. （任意）Taubin 初期値からの geometric refine  
4. （任意）平面の robust 推定、または両手弧  
5. 記事に「精密化手法名」と既知の短弧バイアスへの一言を追記

---

## 6. 参考リンク

- Zenn 記事: https://zenn.dev/izm/articles/b98535429ea1c9  
- Al-Sharadqah & Chernov (2009): https://doi.org/10.1214/09-ejs419  
- Al-Sharadqah & Chernov (2015): https://doi.org/10.1214/14-ejs971  
- Nurunnabi et al., robust 3D circle fitting (2018): https://doi.org/10.1016/j.patcog.2018.04.010  
- Torr & Zisserman, MLESAC/MSAC（ロバスト推定の文脈）
