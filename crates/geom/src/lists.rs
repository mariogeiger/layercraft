//! Lists of items by key, packed in one array: a grid's buckets, a graph's neighbours.
//!
//! List `k` is `items[starts[k]..starts[k + 1]]`, so that consecutive lists read as one slice.
//! They are built by counting each key's items, summing the counts into the starts and filling
//! each list in the order its items come.

/// Lists of items by key, packed in one array.
#[derive(Debug, Clone, Default)]
pub struct Lists<T> {
    starts: Vec<usize>,
    items: Vec<T>,
}

impl<T: Copy + Default> Lists<T> {
    /// The lists of `keys` keys, each holding the items of `pairs` under its key in the order they
    /// come; a pair whose key is past the keys is dropped.
    pub fn from_pairs(keys: usize, pairs: impl Iterator<Item = (usize, T)> + Clone) -> Lists<T> {
        let mut starts = vec![0usize; keys + 1];
        for (k, _) in pairs.clone().filter(|&(k, _)| k < keys) {
            if let Some(count) = starts.get_mut(k + 1) {
                *count += 1;
            }
        }
        let mut total = 0;
        for start in starts.iter_mut() {
            total += *start;
            *start = total;
        }
        let mut fill = starts.clone();
        let mut items = vec![T::default(); total];
        for (k, item) in pairs.filter(|&(k, _)| k < keys) {
            if let Some(slot) = fill.get_mut(k)
                && let Some(place) = items.get_mut(*slot)
            {
                *place = item;
                *slot += 1;
            }
        }
        Lists { starts, items }
    }
}

impl<T: Copy> Lists<T> {
    /// The lists given, packed.
    pub fn from_lists(lists: &[Vec<T>]) -> Lists<T> {
        let mut starts = Vec::with_capacity(lists.len() + 1);
        let mut items = Vec::with_capacity(lists.iter().map(Vec::len).sum());
        for list in lists {
            starts.push(items.len());
            items.extend_from_slice(list);
        }
        starts.push(items.len());
        Lists { starts, items }
    }
}

impl<T> Lists<T> {
    /// List `k`, empty past the keys.
    pub fn get(&self, k: usize) -> &[T] {
        self.span(k, k)
    }

    /// Lists `first` to `last` together, as one slice; empty past the keys.
    pub fn span(&self, first: usize, last: usize) -> &[T] {
        match (self.starts.get(first), last.checked_add(1).and_then(|end| self.starts.get(end))) {
            (Some(&a), Some(&b)) => self.items.get(a..b).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// Every item, list after list.
    pub fn items(&self) -> &[T] {
        &self.items
    }
}
